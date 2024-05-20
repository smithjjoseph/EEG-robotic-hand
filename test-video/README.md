# test-video

Uses SVG and jQuery to create videos for use as stimuli for obtaining EEG data

## Tests
**vep:**
Makes use of a alternating checker pattern to stimulate the occipital lobe

**continuous:**
Associates shape/colour with a motor action in this case the opening and closing of a chosen hand
- The chosen hand laid on a flat surface and should start open palm facing upwards
- The enclosing outer shape indicates the time to the next motion
- When the blue square is filled the chosen hand should be opened
- When the red circle is filled the chosen hand should be closed

**gradual:**
Associates a displayed number with a level of closedness of a chosen hand
- The chosen hand laid on a flat surface and should start open palm facing upwards
- The red text ready indicates that the number is about to change
- When the number changes the hand should be moved to the next position