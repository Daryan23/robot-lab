# Was wir hinzugefügt haben — Residual-Training, Visualisierung & reproduzierbare Demos

Dieses Dokument beschreibt, was gegenüber dem Ausgangs-Repo (Commit
*"Nicklas RL Changes from Spain"*) **geändert** und **neu hinzugefügt** wurde,
und wie man die **Demos** und **Tests** selbst laufen lassen kann.

Kurzfassung: Aus dem reinen PPO-Repo ist ein **Residual-Trainingspfad** geworden
(der handgeschriebene Expert bleibt in der Regelschleife, das neuronale Netz
lernt nur eine begrenzte Korrektur), inklusive voller **Visualisierung**,
**Tests** und einem **Ein-Kommando-Reproduktionsskript** für alle Demos.

---

## 1. Konzept in zwei Sätzen

- **PPO von Null** (war schon im Repo): reines Reinforcement Learning, kein
  Expert, das Netz lernt die ganze Steuerung selbst. Auf den kooperativen
  Aufgaben scheitert das (bleibt bei 0 %).
- **Residual** (neu): `aktion = clip(expert(obs) + scale · NN(obs), -1, 1)`. Der
  Expert liefert eine funktionierende Basis, PPO lernt nur eine *begrenzte
  Korrektur* obendrauf. Startet dank Zero-Init genau auf Expert-Niveau und
  verbessert sich von dort.

---

## 2. Setup (einmalig)

Umgebung aus den mitgelieferten Dateien bauen und das Paket editierbar
installieren:

```bash
# Variante A: conda
conda env create -f environment.yml
conda activate robot-lab
pip install -e .

# Variante B: venv + pip
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
pip install -e .
```

> `pip install -e .` ist nötig, damit `import robot_lab_rl` funktioniert
> (src-Layout, siehe `pyproject.toml`). Ohne diesen Schritt schlagen Tests und
> Skripte mit `ModuleNotFoundError` fehl.

---

## 3. Tests ausführen

```bash
pytest tests/
```

Erwartung: alle Tests grün (u. a. `tests/test_residual.py` mit 6 Tests, die die
Residual-Komposition, das Zero-Init-Verhalten und die Action-Space-Form prüfen).
Die Tests brauchen **keine** trainierten Modelle oder Logs.

---

## 4. Demos reproduzieren (ein Kommando)

Das Treiberskript trainiert **pro Demo** sowohl Residual als auch PPO-von-Null,
misst die Expert-Baseline und erzeugt alle Graphen inklusive des direkten
**PPO-vs-Residual-Vergleichs**:

```bash
# Standard-Demos (die kooperativen Aufgaben), volle Länge (~6 min pro Training)
python scripts/run_demos.py

# schneller Pipeline-Test (wenige Timesteps, Graphen dünn, aber alles läuft durch)
python scripts/run_demos.py --quick

# beliebige Schwierigkeitsgrade wählen
python scripts/run_demos.py --difficulties coop_random coop_mixed full
```

Verfügbare Schwierigkeitsgrade: `easy`, `medium`, `full`, `coop_heavy`,
`coop_rotate`, `coop_mixed`, `coop_random`.

**Was dabei entsteht** — pro Demo unter `figures/demo_<difficulty>/`:

| Datei | Inhalt |
|---|---|
| `learning_success_rate.png` | Erfolgsrate über die Zeit + Expert-Baseline-Linie |
| `reward.png` | durchschnittlicher Episoden-Reward |
| `correction.png` | wie stark das Netz den Expert korrigiert (`|Δ|`) |
| `episode_outcomes.png` | Anteil Erfolg / Stillstand / Timeout |
| `overview.png` | alle vier Panels in einem Bild |
| `method_comparison.png` | PPO von Null vs. Residual über die Zeit (Kontext; Trainingskurven sind auf Zufallsaufgaben verrauscht) |
| `final_comparison.png` | **Headline: finale Erfolgsrate deterministisch** (Expert vs. Residual vs. PPO), Balkendiagramm — die vertrauenswürdige Aussage |

Die Trainings-Logs landen unter `runs/…`, die Modelle unter `models/…` (beide
sind git-ignoriert und werden bei jedem Lauf neu erzeugt).

---

## 5. Einzelbefehle (falls man es manuell steuern will)

```bash
# Residual trainieren (Expert bleibt in der Schleife)
python scripts/train_box_push_ppo.py --residual --residual-scale 0.4 \
    --difficulty coop_random --timesteps 250000 --eval-every 5000 --no-full-eval \
    --seed 0 --log-dir runs/coop_random_residual

# PPO von Null trainieren (kein Expert)
python scripts/train_box_push_ppo.py \
    --difficulty coop_random --timesteps 250000 --eval-every 5000 --no-full-eval \
    --seed 0 --log-dir runs/coop_random_scratch

# Lern-Graphen eines Laufs erzeugen
python scripts/plot_training_progress.py \
    --logdir runs/coop_random_residual --output figures/demo_coop_random \
    --expert-baseline 75

# PPO vs. Residual über die Zeit (Trainingskurven, verrauscht auf Zufallsaufgaben)
python scripts/plot_method_comparison.py \
    --run "Residual=runs/coop_random_residual" \
    --run "PPO von Null=runs/coop_random_scratch" \
    --output figures/demo_coop_random --expert-baseline 75

# Belastbares Endergebnis als Balkendiagramm (deterministisch, viele Episoden)
python scripts/plot_final_comparison.py --difficulty coop_random \
    --residual-model models/demo_coop_random_residual/box_push_ppo_final.zip \
    --scratch-model  models/demo_coop_random_scratch/box_push_ppo_final.zip \
    --residual-scale 0.4 --episodes 200 --output figures/demo_coop_random

# TensorBoard live ansehen
tensorboard --logdir runs
```

### Mehrere Seeds → Fehlerbänder (Reproduzierbarkeit zeigen)

Beide Plot-Skripte mitteln mehrere Seed-Läufe zu einer Mittelwert-Linie mit
schattiertem ±1-std-Band:

```bash
for s in 0 1 2 3 4; do
  python scripts/train_box_push_ppo.py --residual --residual-scale 0.4 \
      --difficulty coop_random --timesteps 250000 --eval-every 5000 --no-full-eval \
      --seed "$s" --log-dir "runs/residual_seed$s"
done
python scripts/plot_training_progress.py --logdir runs/residual_seed* \
    --output figures/residual_bands --expert-baseline 58
```

---

## 6. Geänderte Dateien (gegenüber dem Ausgangs-Repo)

| Datei | Änderung |
|---|---|
| `src/robot_lab_rl/rl.py` | `ResidualActionWrapper` (Expert + begrenzte Korrektur), `make_box_push_env(residual=…)`, `zero_init_residual_policy` (Start ≈ Expert) |
| `src/robot_lab_rl/envs/box_push_env.py` | Env liefert jetzt `info["reward_terms"]` — die vollständige 16-Term-Reward-Zerlegung (Verhalten unverändert) |
| `scripts/train_box_push_ppo.py` | `--residual` / `--residual-scale` / `--residual-log-std-init`, `--seed`, `--no-full-eval`, sowie `BoxPushMetricsCallback` mit vielen zusätzlichen TensorBoard-Metriken (`reward_terms/*`, `residual/*`, `box_push/outcome_*`) |
| `scripts/evaluate_box_push.py` | `--residual` zum Evaluieren einer Residual-Policy |
| `scripts/watch_box_push_policy.py` | `--residual` beim Zuschauen |
| `scripts/watch_training_progress.py` | `--residual` beim Zuschauen |
| `requirements.txt`, `environment.yml` | `matplotlib` ergänzt (für die Plot-Skripte) |

## 7. Neu hinzugefügte Dateien

| Datei | Zweck |
|---|---|
| `scripts/plot_training_progress.py` | Report-Graphen eines Laufs (Erfolg/Reward/Korrektur/Ausgang); mehrere Seeds → Fehlerbänder |
| `scripts/plot_method_comparison.py` | Erfolgskurven mehrerer Methoden/Läufe überlagern (PPO vs. Residual) |
| `scripts/plot_final_comparison.py` | Balkendiagramm der finalen deterministischen Erfolgsrate (Expert vs. Residual vs. PPO), viele Episoden |
| `scripts/run_demos.py` | **Treiber**: trainiert pro Demo Residual + PPO-von-Null und erzeugt alle Graphen |
| `tests/test_residual.py` | 6 Tests für den Residual-Mechanismus |
| `CHANGES.md` | dieses Dokument |
| `figures/…` | die von uns erzeugten Beispiel-Graphen (Referenz) |

---

## 8. Wichtigste Ergebnisse

Gemessen **deterministisch über viele Episoden** (100–200), weil `coop_random`
durch die zufälligen Layouts stark schwankt und einzelne Trainingskurven
irreführend sind:

- **`coop_random`: Residual ~84 % > Expert allein ~75 % > PPO von Null 0 %.**
  Residual verbessert den Expert also echt (~7–10 Prozentpunkte), und PPO ohne
  Expert lernt gar nichts. (Frühere Notizen mit „58 % → 80 %" waren
  Klein-Stichproben-Rauschen und sind korrigiert.)
- **`coop_mixed`** bleibt für alle drei bei ~0 % — die Aufgabe braucht einen
  kooperativen Finish-Skill, den eine begrenzte Korrektur nicht liefern kann.
- Die **Trainingskurven** (`method_comparison.png`) zeigen die Lern-Dynamik über
  die Zeit, sind aber verrauscht; die **belastbare Aussage** steht im
  Balkendiagramm `final_comparison.png`.

## 9. Noch offen (nicht Teil dieses Commits)

- **Dezentralisierung**: das NN ist aktuell zentralisiert (ein Netz sieht beide
  Roboter, gibt alle 4 Radbefehle aus). Ziel wäre ein geteiltes Netz pro Roboter
  auf lokaler Beobachtung. Der Expert ist bereits dezentral aufgebaut.
- Distillation (BC/DAgger) als dritte Vergleichskurve.
