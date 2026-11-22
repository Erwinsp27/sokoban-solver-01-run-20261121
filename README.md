[![Competition](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![Rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Erwinsp27/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![Reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Erwinsp27/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

# Sokoban Solver

A push-optimal Sokoban solver for the make.fun competition
`sokoban-solver-01-run-20261121`. It solves each level room by room
with a push-based best-first search, an admissible box-to-goal
matching heuristic, and deadlock pruning, then stitches the room
solutions together with hallway walks and replays the result
internally before writing it out.

## Run

```sh
node main.ts
```

The program takes no arguments. It reads each level from
`<input>/<id>/level.txt` and writes the move sequence to
`<output>/<id>/solution.txt` for `i001`–`i005`, creating the output
directories itself. All work is budgeted by counted expansions, never
by the clock, so repeated runs produce byte-identical output.

## License

MIT — see [LICENSE](LICENSE).
