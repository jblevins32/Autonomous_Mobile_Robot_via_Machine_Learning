# Deep Deterministic Policy Gradient

import numpy as np
import tensorflow as tf
from tensorflow.keras import layers
import gym

# Create the actor and critic models
def create_actor(state_dim, action_dim, action_bound):
    inputs = layers.Input(shape=(state_dim,))
    out = layers.Dense(400, activation="relu")(inputs)
    out = layers.Dense(300, activation="relu")(out)
    outputs = layers.Dense(action_dim, activation="tanh")(out)
    outputs = outputs * action_bound
    model = tf.keras.Model(inputs, outputs)
    return model

def create_critic(state_dim, action_dim):
    state_input = layers.Input(shape=(state_dim,))
    state_out = layers.Dense(16, activation="relu")(state_input)
    state_out = layers.Dense(32, activation="relu")(state_out)

    action_input = layers.Input(shape=(action_dim,))
    action_out = layers.Dense(32, activation="relu")(action_input)

    concat = layers.Concatenate()([state_out, action_out])

    out = layers.Dense(256, activation="relu")(concat)
    outputs = layers.Dense(1)(out)

    model = tf.keras.Model([state_input, action_input], outputs)
    return model

# Hyperparameters
gamma = 0.99
tau = 0.005
buffer_capacity = 50000
batch_size = 64

# Initialize the actor and critic models
state_dim = 4  # Example state dimension
action_dim = 2  # Example action dimension (linear and angular velocity)
action_bound = 1  # Example action bound

actor_model = create_actor(state_dim, action_dim, action_bound)
critic_model = create_critic(state_dim, action_dim)
target_actor = create_actor(state_dim, action_dim, action_bound)
target_critic = create_critic(state_dim, action_dim)

# Optimizers
critic_optimizer = tf.keras.optimizers.Adam(learning_rate=0.002)
actor_optimizer = tf.keras.optimizers.Adam(learning_rate=0.001)

# Experience replay buffer
class ReplayBuffer:
    def __init__(self, capacity, state_dim, action_dim):
        self.capacity = capacity
        self.state_buffer = np.zeros((capacity, state_dim))
        self.action_buffer = np.zeros((capacity, action_dim))
        self.reward_buffer = np.zeros((capacity, 1))
        self.next_state_buffer = np.zeros((capacity, state_dim))
        self.buffer_counter = 0

    def record(self, obs_tuple):
        index = self.buffer_counter % self.capacity
        self.state_buffer[index] = obs_tuple[0]
        self.action_buffer[index] = obs_tuple[1]
        self.reward_buffer[index] = obs_tuple[2]
        self.next_state_buffer[index] = obs_tuple[3]
        self.buffer_counter += 1

    def sample(self):
        record_range = min(self.buffer_counter, self.capacity)
        batch_indices = np.random.choice(record_range, batch_size)
        state_batch = self.state_buffer[batch_indices]
        action_batch = self.action_buffer[batch_indices]
        reward_batch = self.reward_buffer[batch_indices]
        next_state_batch = self.next_state_buffer[batch_indices]
        return state_batch, action_batch, reward_batch, next_state_batch

# Soft update of target network weights
def update_target(target_weights, weights, tau):
    for (a, b) in zip(target_weights, weights):
        a.assign(b * tau + a * (1 - tau))

# Policy function
def policy(state, noise_object):
    sampled_actions = tf.squeeze(actor_model(state))
    noise = noise_object()
    sampled_actions = sampled_actions.numpy() + noise
    legal_action = np.clip(sampled_actions, -action_bound, action_bound)
    return [np.squeeze(legal_action)]

# Training process
@tf.function
def update(state_batch, action_batch, reward_batch, next_state_batch):
    with tf.GradientTape() as tape:
        target_actions = target_actor(next_state_batch, training=True)
        y = reward_batch + gamma * target_critic([next_state_batch, target_actions], training=True)
        critic_value = critic_model([state_batch, action_batch], training=True)
        critic_loss = tf.math.reduce_mean(tf.math.square(y - critic_value))

    critic_grad = tape.gradient(critic_loss, critic_model.trainable_variables)
    critic_optimizer.apply_gradients(zip(critic_grad, critic_model.trainable_variables))

    with tf.GradientTape() as tape:
        actions = actor_model(state_batch, training=True)
        critic_value = critic_model([state_batch, actions], training=True)
        actor_loss = -tf.math.reduce_mean(critic_value)

    actor_grad = tape.gradient(actor_loss, actor_model.trainable_variables)
    actor_optimizer.apply_gradients(zip(actor_grad, actor_model.trainable_variables))

# Main training loop
import pybullet_envs
env = gym.make("MinitaurBulletEnv-v0")  # Example continuous environment

ep_reward_list = []
avg_reward_list = []

# Noise for exploration
class OUActionNoise:
    def __init__(self, mean, std_deviation, theta=0.15, dt=1e-2, x_initial=None):
        self.theta = theta
        self.mean = mean
        self.std_dev = std_deviation
        self.dt = dt
        self.x_initial = x_initial
        self.reset()

    def __call__(self):
        x = self.x_prev + self.theta * (self.mean - self.x_prev) * self.dt + self.std_dev * np.sqrt(self.dt) * np.random.normal(size=self.mean.shape)
        self.x_prev = x
        return x

    def reset(self):
        self.x_prev = self.x_initial if self.x_initial is not None else np.zeros_like(self.mean)

std_dev = 0.2
ou_noise = OUActionNoise(mean=np.zeros(action_dim), std_deviation=std_dev * np.ones(action_dim))

for ep in range(1000):
    prev_state = env.reset()
    episodic_reward = 0

    while True:
        tf_prev_state = tf.expand_dims(tf.convert_to_tensor(prev_state), 0)
        action = policy(tf_prev_state, ou_noise)
        state, reward, done, info = env.step(action)
        buffer.record((prev_state, action, reward, state))
        episodic_reward += reward

        if buffer.buffer_counter > batch_size:
            states, actions, rewards, next_states = buffer.sample()
            update(states, actions, rewards, next_states)
            update_target(target_actor.variables, actor_model.variables, tau)
            update_target(target_critic.variables, critic_model.variables, tau)

        prev_state = state

        if done:
            break

    ep_reward_list.append(episodic_reward)
    avg_reward = np.mean(ep_reward_list[-40:])
    avg_reward_list.append(avg_reward)
    print("Episode * {} * Avg Reward is ==> {}".format(ep, avg_reward))

