# Video Module

This node is divided in two parts. The first one builds the video and is followed by the second part which will process it. As it needs to load the models, it takes a bit of time to initialise it, so the State Manager wait to receive a confirmation that this node is ready to receive some data before going out of Idle.


## Video Builder

Active during the Recording and Transferring, it receives the data sent by the Visual and Audio modules and send back the quantity of data received. Once those step validated, the State Manager switches in Video Building, so this module now builds the video and then tell the State Manager when this is done so that the state manager knows that the video process can start.

## Video Processor

We now have a video and only need to use the code related to [[1]](#1) to process it. Using the images for the speech recognition, it first searches the face landmarks in every images of the video. Then it uses them to crop the mouth region in every frames and finally send those mouth region in the network. The network returns the wanted prediction of the speech said in the video.

## References
<a id="1">[1]</a> 
Ma, Pingchuan and Petridis, Stavros and Pantic, Maja (2021). 
[End-to-end audio-visual speech recognition with conformers](https://github.com/mpc001/Visual_Speech_Recognition_for_Multiple_Languages)
ICASSP 2021-2021 IEEE International Conference on Acoustics, Speech and Signal Processing (ICASSP), 7613-7617.
