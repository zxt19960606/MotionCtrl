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
import collections


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

class MovingAverageFilter:
    """滑动平均滤波器"""

    def __init__(self, window_size):
        if window_size <= 0:
            window_size = 1
        # if
        self.window = collections.deque(maxlen = window_size)
    # def __init__
    def fill_window(self, val):
        while len(self.window) < self.window.maxlen:
            self.window.append(val)
        # while
    # def

    def update(self, value):
        if len(self.window) == 0:
            self.fill_window(0)
        # if

        self.window.popleft()
        self.window.append(value)
        res = sum(self.window) / len(self.window)
        return res
    # def update
# class MovingAverageFilter


def trapezoidal_integration(v, T, x0 = 0):
    """梯形积分法"""
    x = np.zeros(len(v))
    x[0] = x0
    for i in range(1, len(v)):
        avg_v = (v[i] + v[i - 1]) / 2
        x[i] = x[i - 1] + avg_v * T
    return x
# def trapezoidal_integration

def trapezoidal_diff(v, T, a0 = 0):
    """梯形差分法"""
    a = np.zeros(len(v))
    a[0] = a0
    for i in range(1, len(v)):
        a[i] = (v[i] - v[i - 1]) / T
    return a
# def trapezoidal_integration

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
        V = max(Vm) * 2
        for i in range(len(T)):
            if T[i] != 0:
                V = min(V, np.abs(Vm[i] / T[i])) #路径最大速度
                Am_act[i] = min(Am_act[i], np.abs(Vm[i] / T1[i])) #实际最大轴加速度
            # if
        # for
        Vm_act = V * T # 实际单轴最大速度
        T_total = L / V # 运行时间
        T1_act = np.abs(Vm_act / Am_act) #实际后加减速时间
        # 后加减速处理
        a0 = Vm_act[0] * np.ones(round(T_total / Ts))
        a1 = Vm_act[1] * np.ones(round(T_total / Ts))
        a2 = np.zeros(round(max(T1_act) / Ts))
        a0_2 = np.concatenate([a0, a2])
        a1_2 = np.concatenate([a1, a2])
        V_old = np.column_stack([a0_2, a1_2])
        idx = round(T_total / Ts) + round(max(T1_act) / Ts)
        t_x = np.arange(1, idx+1, 1)
        # 滤波计算
        ma_filter_x = MovingAverageFilter(round(T1_act[0] / Ts))
        ma_filter_y = MovingAverageFilter(round(T1_act[1] / Ts))
        Vt_x = [ma_filter_x.update(v) for v in V_old[:,0]]
        Vt_y = [ma_filter_y.update(v) for v in V_old[:,1]]
        Pt_x = trapezoidal_integration(Vt_x, Ts, Ps[0])
        Pt_y = trapezoidal_integration(Vt_y, Ts, Ps[1])
        At_x = trapezoidal_diff(Vt_x, Ts)
        At_y = trapezoidal_diff(Vt_y, Ts)

        profile = MotionProfile(
            t=t_x,
            At=np.column_stack([At_x, At_y]),
            Vt=np.column_stack([Vt_x, Vt_y]),
            Pt=np.column_stack([Pt_x, Pt_y]),
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

def concatenate_with_overlap(arr1, arr2, overlap):
    # 确保重叠长度有效
    if overlap < 0:
        raise ValueError("重叠长度不能为负数")
    if len(arr1) < overlap or len(arr2) < overlap:
        raise ValueError("重叠长度超过数组长度")
    # if

    # 分割数组
    part1_non_overlap = arr1[:-overlap]
    part1_overlap = arr1[-overlap:]
    part2_overlap = arr2[:overlap]
    part2_remain = arr2[overlap:]

    # 重叠部分相加
    overlapped_sum = part1_overlap + part2_overlap

    # 拼接结果
    result = np.concatenate((part1_non_overlap, overlapped_sum, part2_remain))
    return result
# def concatenate_with_overlap

def concatenate_2d_with_overlap_cols(arr1, arr2, overlap):
    if overlap < 0:
        raise ValueError("重叠长度不能为负数")
    if arr1.shape[0] < overlap or arr2.shape[0] < overlap:
        raise ValueError("重叠长度超过数组行数")
    if arr1.shape[1] != arr2.shape[1]:
        raise ValueError("数组列数必须一致")

    part1_non_overlap = arr1[:-overlap, :]
    part1_overlap = arr1[-overlap:, :]
    part2_overlap = arr2[:overlap, :]
    part2_remain = arr2[overlap:, :]

    overlapped_sum = part1_overlap + part2_overlap
    result = np.vstack([part1_non_overlap, overlapped_sum, part2_remain])
    return result
# def concatenate_2d_with_overlap_cols


def path_overlap(prof1: MotionProfile, prof2: MotionProfile, overlap) -> MotionProfile:
    t = concatenate_with_overlap(prof1.t, prof2.t, overlap)
    a = concatenate_2d_with_overlap_cols(prof1.At, prof2.At, overlap)
    v = concatenate_2d_with_overlap_cols(prof1.Vt, prof2.Vt, overlap)
    # p = concatenate_2d_with_overlap_cols(prof1.Pt, prof2.Pt, overlap)
    p_x = trapezoidal_integration(v[:, 0], 0.001, 0)
    p_y = trapezoidal_integration(v[:, 1], 0.001, 0)
    p = np.column_stack([p_x, p_y])
    return MotionProfile(t=t, At=a, Vt=v, Pt=p)
# def vel_overlap




# ======================
# 示例使用
# ======================
if __name__ == "__main__":
    # 配置参数
    config = AxisConfig(x=MotionConfig(Vm=10000/60.0, Am=10000.0, Dm=10000.0, Jm=0.0, ADCAI_T1=0.2, ADCAI_T2=0.0),
                        y=MotionConfig(Vm=6000/60.0, Am=5000.0, Dm=5000.0, Jm=0.0, ADCAI_T1=0.2, ADCAI_T2=0.0))

    # 初始化运动规划器
    planner = MotionPlanner(profile_type=ADCAI_T_Profile)

    # 执行运动规划
    profile_1 = planner.plan_trajectory(Ps=np.array([0.0, 0.0]), Pe=np.array([100.0, 0.0]), cfg=config, Ts=0.001)
    profile_2 = planner.plan_trajectory(Ps=np.array([100.0, 0.0]), Pe=np.array([100.0, 80.0]), cfg=config, Ts=0.001)
    profile_3 = planner.plan_trajectory(Ps=np.array([100.0, 80.0]), Pe=np.array([45.0, 80.0]), cfg=config, Ts=0.001)
    profile_4 = planner.plan_trajectory(Ps=np.array([45.0, 80.0]), Pe=np.array([80.0, 30.0]), cfg=config, Ts = 0.001)
    profile_5 = planner.plan_trajectory(Ps=np.array([80.0, 30.0]), Pe = np.array([0.0, 0.0]), cfg=config, Ts=0.001)

    # 轨迹重叠
    overlap = int(0.6 * config.x.ADCAI_T1 * 1000) # 叠加时间
    profile_overlap_1 = path_overlap(profile_1, profile_2, overlap)
    profile_overlap_2 = path_overlap(profile_overlap_1, profile_3, overlap)
    profile_overlap_3 = path_overlap(profile_overlap_2, profile_4, overlap)
    profile_overlap_4 = path_overlap(profile_overlap_3, profile_5, overlap)

    ##################################  拼接数据 ##############################################
    At = np.vstack((profile_1.At, profile_2.At, profile_3.At, profile_4.At, profile_5.At))
    Vt = np.vstack((profile_1.Vt, profile_2.Vt, profile_3.Vt, profile_4.Vt, profile_5.Vt))
    Pt = np.vstack((profile_1.Pt, profile_2.Pt, profile_3.Pt, profile_4.Pt, profile_5.Pt))
    At_lap = profile_overlap_4.At
    Vt_lap = profile_overlap_4.Vt
    Pt_lap = profile_overlap_4.Pt
    #######
    path_vel = (Vt[:, 0] ** 2 + Vt[:, 1] ** 2) ** 0.5
    path_vel_lap = (Vt_lap[:, 0] ** 2 + Vt_lap[:, 1] ** 2) ** 0.5
    #######################################################################################

    # fig, axs = plt.subplots(3, 1, figsize = (10, 8))
    # axs[0].plot(Vt[:,0] * 60, label = 'vel x', marker = '.', markersize = 4, linestyle = '-')
    # axs[0].plot(Vt[:,1] * 60, label = 'vel y', marker = '.', markersize = 4, linestyle = '-')
    # axs[0].plot(path_vel * 60, label = 'vel path', marker = '.', markersize = 4, linestyle = '-')
    # plt.grid()
    # plt.title('axis Velo')
    # plt.ylabel('v(mm/min)')
    # plt.legend()
    # # 使用mplcursors 添加鼠标事件监听功能
    # cursor(axs[0], hover = True)
    #
    # #############
    # axs[1].plot(At[:, 0], label = 'acc x', marker = '.', markersize = 4, linestyle = '-')
    # axs[1].plot(At[:, 1], label = 'acc y', marker = '.', markersize = 4, linestyle = '-')
    # plt.grid()
    # plt.title('axis Acc')
    # plt.ylabel('a(mm/s^2)')
    # plt.legend()
    # # 使用mplcursors 添加鼠标事件监听功能
    # cursor(axs[1], hover = True)
    #
    # ######################
    # axs[2].plot(Vt_lap[:, 0] * 60, label = 'vel x', marker = '.', markersize = 4, linestyle = '-')
    # axs[2].plot(Vt_lap[:, 1] * 60, label = 'vel y', marker = '.', markersize = 4, linestyle = '-')
    # axs[2].plot(path_vel_lap * 60, label = 'vel path', marker = '.', markersize = 4, linestyle = '-')
    # plt.grid()
    # plt.title('lap axis Velo')
    # plt.ylabel('v(mm/min)')
    # plt.legend()
    # # 使用mplcursors 添加鼠标事件监听功能
    # cursor(axs[2], hover = True)

    fig, ax = plt.subplots()
    ax.plot(Vt[:, 0] * 60, label = 'vel x', marker = '.', markersize = 4, linestyle = '-')
    ax.plot(Vt[:, 1] * 60, label = 'vel y', marker = '.', markersize = 4, linestyle = '-')
    ax.plot(path_vel * 60, label = 'vel path', marker = '.', markersize = 4, linestyle = '-')
    plt.grid()
    plt.title('axis Velo')
    plt.ylabel('v(mm/min)')
    plt.legend()
    # 使用mplcursors 添加鼠标事件监听功能
    cursor(ax, hover = True)

    fig, ax = plt.subplots()
    ax.plot(Vt_lap[:, 0] * 60, label = 'lap vel x', marker = '.', markersize = 4, linestyle = '-')
    ax.plot(Vt_lap[:, 1] * 60, label = 'lap vel y', marker = '.', markersize = 4, linestyle = '-')
    ax.plot(path_vel_lap * 60, label = 'lap vel path', marker = '.', markersize = 4, linestyle = '-')
    plt.grid()
    plt.title('lap axis Velo')
    plt.ylabel('v(mm/min)')
    plt.legend()
    # 使用mplcursors 添加鼠标事件监听功能
    cursor(ax, hover = True)


    # 程序搭接后的插补点
    fig, ax = plt.subplots()
    ax.plot([0, 100, 100, 45, 80, 0], [0, 0, 80, 80, 30, 0], label = 'NC contour', marker = '.', markersize = 4,
            linestyle = '-')
    ax.plot(Pt_lap[:, 0], Pt_lap[:, 1], label = 'Lap contour', marker = '.', markersize = 4, linestyle = '-')
    plt.grid()
    plt.axis('equal')
    plt.title('XY Contour')
    plt.xlabel('x(mm)')
    plt.ylabel('y(mm)')
    plt.legend()
    # 使用mplcursors 添加鼠标事件监听功能
    cursor(ax, hover = True)

    # 结果可视化
    # visualizer = MotionVisualizer(profile)
    # visualizer.plot_kinematics()
    plt.show()

# if main()
