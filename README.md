# Sokoban Solver — make.fun sokoban-solver-01-run-20261121

Push-optimal Sokoban solver for the make.fun bounty
[Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121).

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Boyoke-Encok/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Boyoke-Encok/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

## How it works

Every level is a 1-wide hallway with 1–4 rooms off it, and a box pushed out
of its room can never come back — so rooms are independent and a level's
push-optimal count is the sum of its rooms' optima.

`main.ts` (Node 22, no dependencies, deterministic) does three things:

1. **Decompose** — splits the level into the hallway row and the room
   components hanging off it, one door each.
2. **Solve each room push-optimally** — A* over pushes, starting with the
   player on the room's door cell. The heuristic is the minimum-cost
   matching of boxes to goals by true shortest-path distance; dead states
   are pruned with corner, wall-segment and 2×2-block deadlock checks.
   The goal state also requires the player to be able to walk back out
   of the door, so room solutions stitch together cleanly.
3. **Stitch** — walks the hallway between doors and concatenates the room
   move strings into one `solution.txt` per instance.

Work budgets are fixed node counts (never the clock), neighbour order is
fixed, and there is no randomness, so repeated runs are byte-identical.

## Files

- `main.ts` — the solver (reads `/input/<id>/level.txt`, writes
  `/output/<id>/solution.txt`)
- `README.md` — this file
- `LICENSE` — MIT
- `.makefun/owner.json` — payout wallet claim file
