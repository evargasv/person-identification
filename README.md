# Person Identification

## Motivation

Person identification is the problem of determining the identity of a person from a closed set of candidates. Conventional biometric systems use person-specific models tested on the same modalities on which they are trained. The modality can be audio, visual or a fusion of audio and visual (bimodal). Recently there has been significant interest in multi-modal human computer interfaces, especially audio-visual (AV) systems for applications in areas such as banking, and security systems.

## Audio Cues

For each speech frame of around i.e. 30 ms with overlap, a set of Mel-Frequency Cepstrum Coefficients (MFCC) is computed. These are result of a cosine transform of the logarithm of the short-term power spectrum expressed on a mel-frequency scale. This set of coefficients is called an acoustic vector.
Therefore each input utterance is transformed into a sequence of acoustic vectors. Extracting the MFCC coefficients shows how those acoustic vectors can be used to represent and recognize the voice characteristic of the speaker.

<p align="center">
  <img src="/img/Audio_Spectrum.png" width="300px">
</p>

## Visual Cues

A set of visual features were computed, in order to have a particular signature that allows the identification of a person. It is composed by different soft biometric cues, extracted from the depth data acquired using a RGB-D sensor, i.e. [Kinect 2](https://developer.microsoft.com/en-us/windows/kinect/). In total 12 visual features were extracted.

### Sekeleton-based

This set of features could be defined as cues based on the combination of distances among joints, distances between the floor plane and all the possible joints.

<p align="center">
  <img src="/img/skel_dist.png" width="300px">
</p>

### Surface-based

The surface-based features refers to geodesic distances on the mesh surface computed from different joint pairs.

<p align="center">
  <img src="/img/surf_dist.png" width="300px">
</p>


## Contributors

- [Tatiana Lopez-Guevara](https://github.com/ZePoLiTaT)
- Dina Youakim
- Ahmed Karam Eldaly 

## References

- Barbosa, IgorBarros; Cristani, Marco; Del Bue, Alessio; Bazzani, Loris; Murino, Vittorio, “Re-identification with RGB-D Sensors”, Computer Vision – ECCV 2012. Workshops and Demonstrations, Lecture Notes in Computer Science, 2012.

- Mogelmose, A; Bahnsen, C.; Moeslund, T.B.; Clapes, A; Escalera, S., "Tri-modal Person Re-identification with RGB, Depth and Thermal Features," Computer Vision and Pattern Recognition Workshops (CVPRW), 2013 IEEE Conference on , vol., no., pp.301,307, 23-28 June 2013.

- Logan, B., 2000, October. Mel frequency cepstral coefficients for music modeling. In ISMIR (Vol. 270, pp. 1-11).

