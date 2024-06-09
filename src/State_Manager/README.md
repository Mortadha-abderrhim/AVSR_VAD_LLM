# State Manager

This module controls the data flow and decision-making of the state transition in the architecture. There are three different timelines/modes, the **normal**, the **experiment** and the **evaluation**. Their differences can be seen in the following Figure and is explained in the next paragraphs. During the normal timeline, we want to understand the user speech using live streaming of the camera and the microphone in real time. Starting from Idle state, we can jump to the Recording state as soon as everything is ready, which can take a few seconds because of the loading of the neural networks used for face tracking and lip reading. We can then stop the recording so that the Transferring state start. The latter ensures that all the data are well arrived before starting the Video Building state. Once that the Video is built, the system start its last task which is the Video Processor state.

While in experiment timeline, it will stay the same except that it will skip the video process. The goal is to only record the live streaming to collect data that can be used later.

And finally, the evaluation timeline is used to evaluate the performances on some videos in all the system. It is important to make it as if the videos are the input of the Visual and Audio modules. The videos can for example be the ones collected during experiment. During evaluation, the Idle and Recording states will be replaced by the Evaluation_idle and Evaluation_recording states. This gives the information to the Visual and Audio modules that they should use a video file as input instead of the live stream.

![States timeline](https://github.com/CHILIpReading/State_Manager/blob/824f39507d35611cff8ccbc7bda052959227f697/images/states.png)
Shema of the flow between the states. Circles are states, rectangle are conditions to switch states and diamonds are conditions to determine which is the following state. The start is at the top left, it is not a state.

To sum up, the state manager receives informations that help it to determine if it has to change the state of the system or not.
