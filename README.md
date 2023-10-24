# Blob-detection-and-Tracking
Blob detection, tracking and serial communication to esp32 through IBus protocol using an OpenMV compatible edge camera device

## The logic flow of the blob tracking:
1. Initialization:
  1. find a reference blob by constraining the minimum roundness, density, and of course color
  2. initialize a feature vector and a history list of blobs based on the reference
    1. *feature vector*: a vector of {five} values marking the **average** center x, y position, width, height, and blob direction of the tracked blob history
    2. *history*: a list of blobs that the algorithm considers as the same blob through multiple frames
3. Update: check the feature vector of the tracked blob against new detected blobs in terms of 1-norm/2-norm feature vector distance
  1. if the history list is full, pop the oldest, push the latest. Otherwise populate it.
  2. if the new blob feature vector is too far from the tracked blob, mark the tracked blob with an additional missed track
  3. if the tracked blob has too many missed tracks, reset the tracked blob history and feature vector, then go to step 1.ii.

## Parameters
This is a test example. However, there are several parameters you can tune to change the tracking behavior:
1. `thresholds`: it determines the color to detect and track
2. In function `find_reference`, `density_threshold` and `roundness_threshold=0.4`: they determine how strictly dense and round the initial blob needs to be.
3. In initialization of the `TrackedBlob` object currently in function `find_reference`,
  1. `norm_level` controls the distance type, 1-norm is faster and 2-norm is smoother (try both and feel the difference)
  2. `feature_dist_threshold` controls how strict a feature vector match needs to be
  3. `window_size` controls how long a history of tracked blobs the user wants to keep. Higher value gives a smoother but more draggy track

## Pending:
1. `id`: we would like to identify/distinguish different blobs.
