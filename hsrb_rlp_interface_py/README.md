Overview
=========

提供機能
---------

Pythonからhsrb_robot_local_planner_nodeへの実行指令インターフェースを提供します。
また、インタラクティブな操作を提供するシェル機能も利用することができます。


How to use
===========

インタラクティブなシェルとして使う方法と、Pythonプログラムの一部として使う方法があります。

インタラクティブシェル
-------------------

対話式にコマンドを実行させることができます。

```bash
$ irlp
```

```bash
RobotLocalPlanner Interactive Shell 0.0.0


        ____  __    ____
       / __ \/ /   / __ \
      / /_/ / /   / /_/ /
     / _, _/ /__ / ____/
    /_/ /_/_____/_/

In [1]: rlp.move_to_go()

In [2]:
Do you really want to exit ([y]/n)? y
Leaving RobotLocalPlanner Interactive Shell
```

ライブラリとしての利用
--------------------

ライブラリとして使う場合には、`robot_local_planner`モジュールをインポートして下さい。

以下に腕関節を動かす簡単な例を記します。

```bash
import math
from hsrb_rlp_interface_py import robot_local_planner

with robot_local_planner.RobotLocalPlanner() as rlp:
    goals = {
        'arm_lift_joint': 0.5,
        'arm_flex_joint': math.radians(-90)
    }
    rlp.move_to_joint_positions(goals)
```