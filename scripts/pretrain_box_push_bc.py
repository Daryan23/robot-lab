import argparse
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from torch.utils.tensorboard import SummaryWriter

from robot_lab_rl.rl import DEFAULT_LOG_DIR, DEFAULT_MODEL_DIR, make_box_push_env


def make_bc_model(difficulty: str = "easy", max_steps: int = 800) -> PPO:
    env = DummyVecEnv([lambda: make_box_push_env(difficulty=difficulty, max_steps=max_steps)])
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
    distribution = model.policy.get_distribution(observations)
    return distribution.distribution.mean


def train_behavior_cloning(
    *,
    demo_path: Path,
    model_dir: Path,
    log_dir: Path,
    epochs: int,
    batch_size: int,
    checkpoint_every: int,
    learning_rate: float,
) -> tuple[float, float]:
    data = np.load(demo_path)
    observations = data["observations"].astype(np.float32)
    actions = data["actions"].astype(np.float32)
    if observations.ndim != 2 or observations.shape[1] != 18:
        raise ValueError(f"Expected observations shape (N, 18), got {observations.shape}")
    if actions.ndim != 2 or actions.shape[1] != 4:
        raise ValueError(f"Expected actions shape (N, 4), got {actions.shape}")

    model_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    model = make_bc_model()
    model.policy.set_training_mode(True)
    optimizer = torch.optim.Adam(model.policy.parameters(), lr=learning_rate)
    writer = SummaryWriter(str(log_dir))

    device = model.device
    observations_tensor = torch.as_tensor(observations, device=device)
    actions_tensor = torch.as_tensor(actions, device=device)
    initial_loss = evaluate_loss(model, observations_tensor, actions_tensor, batch_size)

    rng = np.random.default_rng(7)
    global_step = 0
    for epoch in range(1, epochs + 1):
        indices = rng.permutation(len(observations))
        epoch_losses: list[float] = []
        for start in range(0, len(indices), batch_size):
            batch_indices = indices[start : start + batch_size]
            batch_observations = observations_tensor[batch_indices]
            batch_actions = actions_tensor[batch_indices]
            predicted_actions = policy_action_mean(model, batch_observations)
            loss = torch.nn.functional.mse_loss(predicted_actions, batch_actions)

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.policy.parameters(), 0.5)
            optimizer.step()

            global_step += 1
            epoch_losses.append(float(loss.detach().cpu()))

        average_epoch_loss = float(np.mean(epoch_losses))
        writer.add_scalar("bc/loss", average_epoch_loss, epoch)
        print(f"epoch={epoch:03d} bc_loss={average_epoch_loss:.6f}")
        if checkpoint_every > 0 and epoch % checkpoint_every == 0:
            model.save(model_dir / f"bc_checkpoint_epoch_{epoch:03d}")

    final_loss = evaluate_loss(model, observations_tensor, actions_tensor, batch_size)
    writer.add_scalar("bc/initial_loss", initial_loss, 0)
    writer.add_scalar("bc/final_loss", final_loss, epochs)
    writer.close()
    model.save(model_dir / "box_push_bc_pretrained")
    model.env.close()
    print(f"Initial BC loss: {initial_loss:.6f}")
    print(f"Final BC loss: {final_loss:.6f}")
    print(f"Saved BC model to {model_dir / 'box_push_bc_pretrained.zip'}")
    return initial_loss, final_loss


def evaluate_loss(
    model: PPO,
    observations: torch.Tensor,
    actions: torch.Tensor,
    batch_size: int,
) -> float:
    losses: list[float] = []
    with torch.no_grad():
        for start in range(0, len(observations), batch_size):
            predicted_actions = policy_action_mean(model, observations[start : start + batch_size])
            loss = torch.nn.functional.mse_loss(predicted_actions, actions[start : start + batch_size])
            losses.append(float(loss.cpu()))
    return float(np.mean(losses))


def main() -> None:
    parser = argparse.ArgumentParser(description="Behavior-clone PPO from box-push expert demonstrations.")
    parser.add_argument("--demo-path", type=Path, default=Path("data/box_push_demos.npz"))
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR.parent / "box_push_bc")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--checkpoint-every", type=int, default=5)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    args = parser.parse_args()

    train_behavior_cloning(
        demo_path=args.demo_path,
        model_dir=args.model_dir,
        log_dir=args.log_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        checkpoint_every=args.checkpoint_every,
        learning_rate=args.learning_rate,
    )


if __name__ == "__main__":
    main()
