# Uses mediapipe and scikitlearn to do emotion detection
# https://www.youtube.com/watch?v=h0LoewzGzhc

# Will require an incremental delta control method

# IDEA 1:
# Neutral = do nothing
# Happy = incrementally open
# Sad = incrementally close

# IDEA 2:
# Open mouth = unlock movement ability
# Right eye = incrementally open
# Left eye = incrementally close

# Generate dataset images (happy, sad, neutral) using AI image generators
