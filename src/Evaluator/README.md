# Evaluator

The goal of this part is simply to make a smooth process to evaluate how the system performs on videos. At initialisation it gets the path of all the videos that it should evaluate and will then make the system evaluate them one per state cycles. During the state cycles it takes the timing and accuracy measures wanted. At every end of cycles, it saves the results in a csv file. For the purpose of this report, the saved elements are the following:

- Participant: Keeps track if it is an English native speaker.
- Preprocessing: Keeps track of the face landmarks detection algorithm used.
- Recording [s]: The time it took to record the video. It is close to the duration of the video.
- Transferring [s]: The time it took to end the transfer after that the recording ended.
- Video Building [s]: The time it took to build the mp4 video using the data received.
- Video Processing [s]: The time it takes for the video building, face tracking and inference.
- WER: World Error Rate
- Nb word: Number of worlds in the ground truth sentence. Allows to compute the WER of all sentences together.
- CER: Character Error Rate
- Nb character: Number of characters in the ground truth sentence. Allows to compute the CER of all sentences together.
- Ground truth: The sentence read in the video.
- Prediction: The prediction that the algorithm made on the speech of the video.
