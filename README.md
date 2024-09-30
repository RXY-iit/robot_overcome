
# Self-Programming Robots: Overcoming the Unexpected with AI-driven Autonomous Code Generation

This repository accompanies the paper under review titled "Are We Close to Realizing Self-Programming Robots That Overcome the Unexpected?" In this work, we are exploring how robots equipped with Large Language Models (LLMs) can autonomously modify their operational code in response to unforeseen challenges during task execution. Specifically, we investigate how the OpenAI o1-preview model enables a mobile robot to analyze failure scenarios and autonomously generate new code to successfully overcome obstacles without human intervention.

### Video Demonstrations

1. **[First Scenario: Default Environment and Robot Behavior](https://youtu.be/hwbhQHJ-iIg)**
   - In this video, the robot follows its pre-programmed path until it encounters an obstacle. The robot halts after detecting the object and, since it is not removed, it reaches the 20-second timeout. After the timeout, the o1-preview model-generated code is executed, enabling the robot to autonomously navigate around the obstacle and reach its goal.

2. **[Second Scenario: Different Obstacle Placements](https://youtu.be/57eOCXrZY-0) & [Third Scenario](https://youtu.be/-Dj_W1FMp8s)**
   - These two videos demonstrate the same generated code from the o1-preview model applied to two different obstacle setups. In both cases, the robot successfully avoids the obstacles and completes the task, highlighting the flexibility of the generated solution.

### Full Inputs and Outputs

The repository contains the full inputs and outputs from both the **OpenAI o1-preview** model and the **GPT-4o** model. These files demonstrate how the o1-preview model successfully reasoned about the task failure and generated a new code that allowed the robot to autonomously adapt and continue towards its goal. Also, it contains the summary of the Chain-of-Thought(CoT) by o1-preview. 

- **Model Outputs**: You can find the full structured prompt and model-generated codes under the `/outputs` folder in the repository's home directory.

**Observation:** Although the GPT-4o model demonstrated a good level of reasoning in analyzing the problem, it was unable to solve the issue in a single attempt. In the GPT-4o trial, the robot turns in place after detecting the obstacle, but stops after no obstacle is detected, without reaching the goal.


