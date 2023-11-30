#include "PSO.h"

ParticleSwarm Swarm;


// 初始化粒子群
void InitializeSwarm(ParticleSwarm *swarm) {
//    srand((unsigned int)time(NULL));
    for (int i = 0; i < NUM_PARTICLES; i++) {
        for (int j = 0; j < NUM_DIMENSIONS; j++) {
            swarm->particles[i].position[j] = P_MIN + ((float)rand() / RAND_MAX) * (P_MAX - P_MIN);
            swarm->particles[i].velocity[j] = V_MIN + ((float)rand() / RAND_MAX) * (V_MAX - V_MIN);
            swarm->particles[i].pBest[j] = swarm->particles[i].position[j];
        }
        swarm->particles[i].fitness = INFINITY;
        swarm->particles[i].pBestFitness = INFINITY;
    }
    for (int j = 0; j < NUM_DIMENSIONS; j++) {
        swarm->gBest[j] = swarm->particles[0].position[j];
    }
    swarm->gBestFitness = INFINITY;
}

// 更新粒子的速度
void UpdateVelocity(Particle *particle, const ParticleSwarm *swarm) {
    for (int j = 0; j < NUM_DIMENSIONS; j++) {
        float r1 = ((float)rand() / RAND_MAX);
        float r2 = ((float)rand() / RAND_MAX);
        particle->velocity[j] = W * particle->velocity[j] +
                                C1 * r1 * (particle->pBest[j] - particle->position[j]) +
                                C2 * r2 * (swarm->gBest[j] - particle->position[j]);
        // 限制速度
        if (particle->velocity[j] > V_MAX) {
            particle->velocity[j] = V_MAX;
        } else if (particle->velocity[j] < V_MIN) {
            particle->velocity[j] = V_MIN;
        }
    }
}

// 更新粒子的位置
void UpdatePosition(Particle *particle) {
    for (int j = 0; j < NUM_DIMENSIONS; j++) {
        particle->position[j] += particle->velocity[j];
        // 限制位置
        if (particle->position[j] > P_MAX) {
            particle->position[j] = P_MAX;
        } else if (particle->position[j] < P_MIN) {
            particle->position[j] = P_MIN;
        }
    }
}

// 评估粒子的适应度
void EvaluateParticle(Particle *particle) {
    particle->fitness = fitnessFunc(particle,particle->position[2]);
}

// 更新个体和全局最优值
void UpdateBestValues(Particle *particle, ParticleSwarm *swarm) {
    // 更新个体最优值
    if (particle->fitness < particle->pBestFitness) {
        for (int j = 0; j < NUM_DIMENSIONS; j++) {
            particle->pBest[j] = particle->position[j];
        }
        particle->pBestFitness = particle->fitness;
    }
    // 更新全局最优值
    if (particle->fitness < swarm->gBestFitness) {
        for (int j = 0; j < NUM_DIMENSIONS; j++) {
            swarm->gBest[j] = particle->position[j];
        }
        swarm->gBestFitness = particle->fitness;
    }
}

// 执行PSO优化
void PSOOptimize(ParticleSwarm *swarm) {
    InitializeSwarm(swarm);
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        for (int j = 0; j < NUM_PARTICLES; j++) {
            UpdateVelocity(&swarm->particles[j], swarm);
            UpdatePosition(&swarm->particles[j]);
            EvaluateParticle(&swarm->particles[j]);
            UpdateBestValues(&swarm->particles[j], swarm);
        }
    }
}


float fitnessFunc(Particle *particle,float erro) {
//				float summ=0;
//        for (int j = 0; j < NUM_DIMENSIONS; j++) 
//				{
//         summ += (particle->position[j]*particle->position[j]);
//        }
//				return summ;
    float itae;
    itae+=(0.001f * time_tick * fabs(erro));
    return itae;
}

