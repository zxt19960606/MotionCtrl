# =========================
# -*- coding: utf-8 -*-
# @Time: 2025/3/18 20:54
# @Author：zhangxuetao
# @File：main_exe.py
# =========================
import copy

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Protocol
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from mplcursors import cursor
from abc import ABC, abstractmethod
import math


# ======================
# 类型定义和配置结构
# ======================
@dataclass(frozen=True)
class MotionConfig:
    """
    轴的运动配置
    """
    Vm: float # mm/s
    Am: float # mm/s²
    Dm: float  # mm/s²
    Jm: float = 0.0 # mm/s³
    ADCAI_T1: float = 0.2 # s
    ADCAI_T2: float = 0.0  # s
# class MotionConfig

@dataclass(frozen=True)
class AxisConfig:
    """
    轴配置
    """
    x: MotionConfig
    y: MotionConfig
# class AxisConfig

@dataclass
class MotionProfile:
    """
    运动规划
    """
    t: np.ndarray # 时间序列 (s)
    Pt: np.ndarray # 各轴位置 (mm)
    Vt: np.ndarray # 各轴速度 (mm/s)
    At: np.ndarray # 各轴加速度 (mm/s²)
# class MotionProfile

class AccelerationProfile(ABC):
    """
    加减速曲线抽象基类
    """
    @abstractmethod
    def generate(self, Ps: float, Pe: float, Ts: float) -> MotionProfile:
        pass
    # def generate
# class AccelerationProfile

class ADCAI_T_Profile(AccelerationProfile):
    """
    梯形后加减速算法
    """
    def __init__(self, config: AxisConfig):
        self.cfg = config
    # def __init__

    def generate(self, Ps: np.ndarray, Pe: np.ndarray, Ts: float) -> MotionProfile:
        """
        后加减速，T型规划
        :param Ps: 起点坐标
        :param Pe: 终点坐标
        :param Ts: 插补周期
        :return: 返回运动曲线
        """
        delta = Pe - Ps
        L = np.linalg.norm(delta)
        T = delta / L
        Vm = np.array([self.cfg.x.Vm, self.cfg.y.Vm])
        Am = np.array([self.cfg.x.Am, self.cfg.y.Am])
        T1 = np.array([self.cfg.x.ADCAI_T1, self.cfg.y.ADCAI_T1])
        Am_act = copy.deepcopy(Am)
        V = max(Vm)
        for i in range(len(T)):
            if T[i] != 0:
                V = min(V, Vm[i] / T[i]) #路径最大速度
                Am_act[i] = min(Am_act[i], Vm[i] / T1[i]) #实际最大轴加速度
            # if
        # for
        Vm_act = V * T # 实际单轴最大速度
        T_total = V / L # 运行时间
        T1_act = Vm_act / Am_act #实际后加减速时间
        # 后加减速处理
        V_old = np.zeros(100)
        profile = MotionProfile(
            t=np.array([[1,2,3], [4,5,6]], dtype=np.double),
            At=np.array([[1, 2, 3], [4, 5, 6]], dtype=np.double),
            Vt=np.array([[1, 2, 3], [4, 5, 6]], dtype=np.double),
            Pt=np.array([[1, 2, 3], [4, 5, 6]], dtype=np.double),
        )
        return profile
    # def generate
# class ADCAI_T_Profile

class MotionPlanner:
    """
    运动规划引擎
    """
    def __init__(self, profile_type: type = ADCAI_T_Profile):
        self.profile_type = profile_type
    # def __init__

    def plan_trajectory(self, Ps: np.ndarray, Pe: np.ndarray, cfg: AxisConfig, Ts: float = 0.001 ) -> MotionProfile:
        """
        执行完整的运动规划流程
        """

        planner = self.profile_type(cfg)
        profile = planner.generate(Ps, Pe, Ts)
        return profile
    # def plan_trajectory
# class MotionPlanner

class MotionVisualizer:
    """
    运动可视化类
    """
    def __init__(self, profile: MotionProfile):
        self.profile = profile
    # def __init__

    def plot_kinematics(self, axis: int = 0):
        """绘制运动学曲线"""
        fig, axs = plt.subplots(3, 1, figsize=(10, 8))
        self._plot_axis(axs[0], 'Position (mm)', self.profile.Pt[:,axis])
        self._plot_axis(axs[1], 'Velocity (mm/s)', self.profile.Vt[:,axis])
        self._plot_axis(axs[2], 'Acceleration (mm/s²)', self.profile.At[:,axis])
        plt.tight_layout()
    # def plot_kinematics

    def _plot_axis(self, ax, ylabel, data):
        ax.plot(self.profile.t, data, marker='.', markersize=2)
        ax.set_ylabel(ylabel)
        ax.grid(True)
        cursor(ax, hover=True)
    # def _plot_axis

    def plot_trajectory_3d(self):
        """三维轨迹可视化"""
        if self.profile.Pt.shape[1] < 3:
            raise ValueError("Not enough axes for 3D visualization")
        # if

        ax = plt.figure().add_subplot(111, projection='3d')
        ax.plot(self.profile.Pt[:,0],
        self.profile.Pt[:,1],
        self.profile.Pt[:,2])
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        plt.title('3D Motion Trajectory')
    # def plot_trajectory_3d
# class MotionVisualizer


# ======================
# 示例使用
# ======================
if __name__ == "__main__":
    # 配置参数
    config = AxisConfig(x=MotionConfig(Vm=10000/60.0, Am=10000.0, Dm=10000.0, Jm=0.0, ADCAI_T1=0.2, ADCAI_T2=0.0),
                        y=MotionConfig(Vm=6000 / 60.0, Am=5000.0, Dm=10000.0, Jm=0.0, ADCAI_T1=0.2, ADCAI_T2=0.0))

    # 初始化运动规划器
    planner = MotionPlanner(profile_type=ADCAI_T_Profile)

    # 执行运动规划
    profile = planner.plan_trajectory(Ps=np.array([0.0, 0.0]), Pe=np.array([80.0, 60.0]), cfg=config, Ts=0.001)

    # 结果可视化
    visualizer = MotionVisualizer(profile)
    visualizer.plot_kinematics()
    plt.show()

# if main()
