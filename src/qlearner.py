# q-learning for discrete action space
import numpy as np
import random
import matplotlib.pyplot as plt

class QLearner:
    def __init__(self, grid, start, goal, learning_rate, gamma, epsilon):
        
        # initialize important parameters
        self.grid = grid
        self.start = start
        self.goal = goal
        self.learning_rate = learning_rate
        self.gamma = gamma
        self.epsilon = epsilon
        self.state_space = grid.shape
        self.action_space = 4  # Up, down, left, right
        self.q_table = np.zeros((*self.state_space, self.action_space)) # build q table of 250x250 size where each index has a q-value for each action from that state
    
    # Generate rewards
    def reward(self, state):
        if state == self.goal:
            return 100  # Large reward for reaching the goal
        elif self.grid[state] == -1:
            return -100  # Large negative reward for hitting an obstacle
        else:
            return -1  # Small negative reward for each step to encourage finding the shortest path
    
    # Generate movements
    def state_observation(self, state, action):
        row, col = state
        if action == 0:  # Up
            row -= 1
        elif action == 1:  # Down
            row += 1
        elif action == 2:  # Left
            col -= 1
        elif action == 3:  # Right
            col += 1
        
        # Ensure new state is within bounds
        row = max(0, min(row, self.grid.shape[0] - 1))
        col = max(0, min(col, self.grid.shape[1] - 1))
        
        return (row, col)
    
    # Main Q-learning update rule
    def update_entry(self, state, action):
        next_state = self.state_observation(state, action) # what is the next state
        reward = self.reward(next_state) # what is the reward of that next state
        
        # Q-value update rule
        best_next_action = np.argmax(self.q_table[next_state]) # what is the best action to take at the next state? (left, right, up, down)
        td_target = reward + self.gamma * self.q_table[next_state][best_next_action]
        td_error = td_target - self.q_table[state][action]
        
        self.q_table[state][action] += self.learning_rate * td_error
        
        return next_state
    
    # Randomly choose to explore or exploit (random choice of direction vs look at q-table) and return the action
    def choose_action(self, state):
        if random.uniform(0, 1) < self.epsilon:
            return random.randint(0, self.action_space - 1)  # Explore
        else:
            return np.argmax(self.q_table[state])  # Exploit - do action that provides best Q-value
    
    def run_q(self, episodes):
        for episode in range(episodes):
            state = self.start
            while state != self.goal:
                action = self.choose_action(state) # Choose an action
                next_state = self.update_entry(state, action)
                state = next_state
                # print(state)

        print("Training completed!")
        print("Q-Table:", self.q_table)
    
    # Plotting path and map by taking the path of best q value
    def trace_path(self):
        state = self.start
        path = [state]
        print("Number of unexplored states:", np.sum(np.sum(np.sum(self.q_table!=0))))

        while state != self.goal:
            action = np.argmax(self.q_table[state])
            state = self.state_observation(state, action)
            if state in path:  # Prevent infinite loops
                break
            path.append(state)
        return path

    def plot_path(self):
        path = self.trace_path()
        path_grid = self.grid.copy()
        
        for step in path:
            path_grid[step] = 3  # Mark the path
        path_grid[self.start] = 1  # Start
        path_grid[self.goal] = 2  # Goal

        plt.figure(figsize=(6, 6))
        plt.imshow(path_grid, cmap='Accent', origin='upper')
        plt.title('Q-Learner Path from Start to Goal')
        plt.show()