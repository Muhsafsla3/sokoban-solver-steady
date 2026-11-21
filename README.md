# Sokoban solver — IDA* per room (steady)

Solver Sokoban push-optimal untuk kompetisi make.fun
`sokoban-solver-01-run-20261121` (akun steady-badger-95).

## Cara kerja

1. Level dipecah di baris hallway (baris `@`) menjadi room-room independen.
2. Tiap room diselesaikan push-optimal dengan **IDA\*** (iterative-deepening
   A*) atas aksi push: bound = pushes + heuristic, bound naik ke f-value
   terkecil yang melebihi bound.
3. Heuristic admissible: min-cost matching box→goal (permutasi brute-force,
   box ≤ 4) memakai jarak all-pairs BFS yang di-precompute per sel.
4. Pruning: dead-square (fixpoint reverse-pull), frozen 2x2, dan
   transposition table per iterasi (boxes + player-region mask).
5. Goal test room: semua box di goal DAN player bisa mencapai sel pintu.
6. Room dijahit kiri→kanan; jalan kaki antar pintu lewat hallway (BFS,
   box = obstacle); tiap push didahului jalan kaki BFS ke belakang box.

## Menjalankan

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` mengekspor `solveLevel(levelText): string` (pure, tanpa IO);
wrapper IO hanya jalan saat dieksekusi langsung sebagai script.
Node 22+, tanpa dependensi, tanpa network, tanpa jam dinding —
budget dihitung dari jumlah expansion, bukan waktu.

## Lisensi

MIT — lihat `LICENSE`.
