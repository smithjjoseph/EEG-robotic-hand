const FRAMES_PER_SEC = 60;
const NUMBER_TIME_SEC = 1.5;

var frames_per_ms = 1000 / FRAMES_PER_SEC;
var target_number_frames = NUMBER_TIME_SEC * FRAMES_PER_SEC * 3/4;
var target_ready_frames = target_number_frames / 3;

var time_counter = 0;
var stage = 0;

function wait(delay_frames) {
    if (time_counter >= delay_frames) {
        time_counter = 0;
        stage += 1;
    } else time_counter += 1;
}

function hide(elem) {
    elem.attr("visibility", "hidden");
}

function show(elem) {
    elem.attr("visibility", "visible");
}

// Start running JQuery once page is loaded otherwise it doesn't work
$(document).ready(function() {
    const ready = $("#ready");
    const number = $("#number");

    function nextSlide(){
        if (stage > 3) stage = 0;

        switch (stage) {
            case 0:
                wait(target_number_frames);
                break;
            case 1:
                show(ready);
                stage += 1;
                break;
            case 2:
                wait(target_ready_frames);
                break;
            case 3:
                hide(ready);
                number.text(parseInt(number.text()) + 1);
                if(number.text() > 5) number.text(0);
                stage += 1;
                break;
            default:
                console.log("ERROR: Stage switch-case out of scope.");
                break;
        }
    }

    setInterval(nextSlide, frames_per_ms)
});