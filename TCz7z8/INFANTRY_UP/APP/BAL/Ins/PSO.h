#ifndef __PSO_H
#define __PSO_H

#include "public.h"

// ����Ⱥ�㷨�Ĳ���
#define NUM_PARTICLES 30    // ��������
#define NUM_DIMENSIONS 3    // ά����������ӦPID��Kp, Ki, Kd
#define NUM_ITERATIONS 100  // ��������
#define W 0.729             // ����Ȩ��
#define C1 1.49445          // ����ѧϰ����
#define C2 1.49445          // ���ѧϰ����
#define V_MAX 0.5           // ����ٶ�
#define V_MIN -0.5          // ��С�ٶ�
#define P_MAX 10.0          // ���������ֵ
#define P_MIN 0.0           // ��������Сֵ

// ���ӽṹ��
typedef struct {
    float position[NUM_DIMENSIONS];  // λ��
    float velocity[NUM_DIMENSIONS];  // �ٶ�
    float pBest[NUM_DIMENSIONS];     // ��������λ��
    float  fitness;                   // ��Ӧ��ֵ
    float pBestFitness;              // ����������Ӧ��ֵ
} Particle;

// ����Ⱥ�ṹ��
typedef struct {
    Particle particles[NUM_PARTICLES];  // ��������
    float gBest[NUM_DIMENSIONS];       // ȫ������λ��
    float gBestFitness;                // ȫ��������Ӧ��ֵ
} ParticleSwarm;

// ��������
void InitializeSwarm(ParticleSwarm *swarm);
void UpdateVelocity(Particle *particle, const ParticleSwarm *swarm);
void UpdatePosition(Particle *particle);
void EvaluateParticle(Particle *particle);
void UpdateBestValues(Particle *particle, ParticleSwarm *swarm);
void PSOOptimize(ParticleSwarm *swarm);
float fitnessFunc(Particle *particle,float erro);
extern ParticleSwarm Swarm;
#endif // PSO_H
