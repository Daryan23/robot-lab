"""Warm-start the shared box-push policy by cloning the geometric expert.

Pipeline (all decentralized, per-robot):
    1. run the expert and record (egocentric obs, that robot's action) pairs,
    2. behavior-clone: train the shared MlpPolicy to reproduce those actions
       (pure supervised MSE -- no reward, no PPO yet),
    3. save a model that train_shared_ppo.py can load with --pretrained-model.

Example:
    python scripts/pretrain_shared_bc.py --epochs 60
    python scripts/train_shared_ppo.py --pretrained-model \
        models/box_push_shared_ppo/shared_bc_pretrained.zip --difficulty full
"""

import argparse
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO

from robot_lab_rl.decentralized import collect_expert_demos, make_shared_box_push_vec_env

DEFAULT_MODEL_DIR = Path("models/box_push_shared_ppo")


def make_shared_bc_model() -> PPO:
    """A PPO whose MlpPolicy matches the shared vec-env spaces (17 -> 2)."""
    env = make_shared_box_push_vec_env(difficulty="easy", max_steps=800)
    return PPO(
        "MlpPolicy",
        env,
        verbose=0,
        n_steps=2048,
        batch_size=512,
        gamma=0.995,
        gae_lambda=0.95,
        ent_coef=0.01,
        learning_rate=2.5e-4,
        clip_range=0.2,
        max_grad_norm=0.5,
        policy_kwargs={"net_arch": [128, 128]},
    )


def policy_action_mean(model: PPO, observations: torch.Tensor) -> torch.Tensor:
    return model.policy.get_distribution(observations).distribution.mean


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--difficulties", nargs="+", default=["easy", "medium", "full"])
    parser.add_argument("--steps-per-difficulty", type=int, default=20_000)
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument("--randomization", type=float, default=0.0)
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    args = parser.parse_args()

    print("Collecting expert demonstrations...")
    observations, actions = collect_expert_demos(
        difficulties=tuple(args.difficulties),
        steps_per_difficulty=args.steps_per_difficulty,
        max_steps=args.max_episode_steps,
        randomization=args.randomization,
    )
    print(f"Collected {len(observations)} (obs, action) pairs.")

    model = make_shared_bc_model()
    model.policy.set_training_mode(True)
    optimizer = torch.optim.Adam(model.policy.parameters(), lr=args.learning_rate)
    device = model.device
    observations_tensor = torch.as_tensor(observations, device=device)
    actions_tensor = torch.as_tensor(actions, device=device)

    rng = np.random.default_rng(7)
    for epoch in range(1, args.epochs + 1):
        indices = rng.permutation(len(observations))
        epoch_losses: list[float] = []
        for start in range(0, len(indices), args.batch_size):
            batch = indices[start : start + args.batch_size]
            predicted = policy_action_mean(model, observations_tensor[batch])
            loss = torch.nn.functional.mse_loss(predicted, actions_tensor[batch])
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.policy.parameters(), 0.5)
            optimizer.step()
            epoch_losses.append(float(loss.detach().cpu()))
        print(f"epoch={epoch:03d} bc_loss={np.mean(epoch_losses):.6f}")

    args.model_dir.mkdir(parents=True, exist_ok=True)
    output_path = args.model_dir / "shared_bc_pretrained"
    model.save(output_path)
    model.env.close()
    print(f"Saved warm-start model to {output_path}.zip")


if __name__ == "__main__":
    main()
