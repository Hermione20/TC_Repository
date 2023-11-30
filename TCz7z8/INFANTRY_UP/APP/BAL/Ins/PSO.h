#ifndef __PSO_H
#define __PSO_H

#include "public.h"

// 粒子群算法的参数
#define NUM_PARTICLES 30    // 粒子数量
#define NUM_DIMENSIONS 3    // 维度数量，对应PID的Kp, Ki, Kd
#define NUM_ITERATIONS 100  // 迭代次数
#define W 0.729             // 惯性权重
#define C1 1.49445          // 个体学习因子
#define C2 1.49445          // 社会学习因子
#define V_MAX 0.5           // 最大速度
#define V_MIN -0.5          // 最小速度
#define P_MAX 10.0          // 参数的最大值
#define P_MIN 0.0           // 参数的最小值

// 粒子结构体
typedef struct {
    float position[NUM_DIMENSIONS];  // 位置
    float velocity[NUM_DIMENSIONS];  // 速度
    float pBest[NUM_DIMENSIONS];     // 个体最优位置
    float  fitness;                   // 适应度值
    float pBestFitness;              // 个体最优适应度值
} Particle;

// 粒子群结构体
typedef struct {
    Particle particles[NUM_PARTICLES];  // 粒子数组
    float gBest[NUM_DIMENSIONS];       // 全局最优位置
    float gBestFitness;                // 全局最优适应度值
} ParticleSwarm;

// 函数声明
void InitializeSwarm(ParticleSwarm *swarm);
void UpdateVelocity(Particle *particle, const ParticleSwarm *swarm);
void UpdatePosition(Particle *particle);
void EvaluateParticle(Particle *particle);
void UpdateBestValues(Particle *particle, ParticleSwarm *swarm);
void PSOOptimize(ParticleSwarm *swarm);
float fitnessFunc(Particle *particle,float erro);
extern ParticleSwarm Swarm;
#endif // PSO_H
