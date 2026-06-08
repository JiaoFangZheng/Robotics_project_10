#!/usr/bin/env python3
"""
实验启动入口 — 选择要运行的对比实验
用法: python3 ~/run_experiments.py [v1|v2|list]
"""
import sys, os

EXPERIMENTS = {
    'v1': {
        'name': 'V1 基础对比 (无起点对齐)',
        'script': 'experiments/v1/run_filter_experiment.py',
    },
    'v2': {
        'name': 'V2 公平起点对齐 (推荐用于论文)',
        'script': 'experiments/v2/run_filter_experiment_v2.py',
    },
}

def main():
    if len(sys.argv) < 2 or sys.argv[1] == 'list':
        print('可用实验:')
        for k, v in EXPERIMENTS.items():
            print(f'  {k}: {v["name"]}')
        print(f'\n用法: python3 ~/run_experiments.py [{"|".join(EXPERIMENTS)}]')
        return

    key = sys.argv[1]
    if key not in EXPERIMENTS:
        print(f'未知实验: {key}')
        return

    exp = EXPERIMENTS[key]
    print(f'启动: {exp["name"]}...')
    os.execlp('python3', 'python3', os.path.expanduser('~/' + exp['script']))

if __name__ == '__main__':
    main()
