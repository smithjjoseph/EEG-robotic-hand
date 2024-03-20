const FRAMES_PER_SEC = 60;
const TARGET_DELAY_SEC = 0.5;
const OPACITY_CHANGE = 0.01;
const OUTER_LEN_CHANGE = 0.5;
const OUTER_RESET_SIZE = 40;

var frames_per_ms = 1000/FRAMES_PER_SEC;
var target_delay_frames = TARGET_DELAY_SEC * FRAMES_PER_SEC;
var time_counter = 0;
var stage = 0;

// Basic wait function needed to workaround setInterval()
function wait(delay_frames) {
    if (time_counter >= delay_frames) {
        time_counter = 0;
        stage += 1
    } else time_counter += 1;
}

function hide(elem) {
    elem.attr("visibility", "hidden");
}

function show(elem) {
    elem.attr("visibility", "visible");
}

function empty(elem) {
    elem.attr("fill-opacity", "0");
}

function fill(elem) {
    elem.attr("fill-opacity", ".8");
}

function fade_in(elem) {
    let new_opacity = parseFloat(elem.css("fill-opacity")) + OPACITY_CHANGE;
    if (new_opacity > 1) {
        new_opacity = 0;
        stage += 1;
    }
    elem.attr("style", "fill-opacity: " + new_opacity);
}

// Start running JQuery once page is loaded otherwise it doesn't work
$(document).ready(function() {
    const outer_square = $("#outer-square");
    const outer_circle = $("#outer-circle");
    const square = $("#square");
    const circle = $("#circle");

    const INIT_OS_X = outer_square.attr("x");
    const INIT_OS_Y = outer_square.attr("y");
    const INIT_OS_L = outer_square.width();
    const INIT_OC_R = outer_circle.attr("r");

    function reset_OS() {
        outer_square.attr("x", INIT_OS_X);
        outer_square.attr("y", INIT_OS_Y);
        outer_square.width(INIT_OS_L);
        outer_square.height(INIT_OS_L);
        show(outer_square);
        stage += 1;
    }

    function reset_OC() {
        outer_circle.attr("r", INIT_OC_R);
        show(outer_circle);
        stage += 1;
    }

    function shrink_OS() {
        let x = parseFloat(outer_square.attr("x")) + OUTER_LEN_CHANGE;
        let y = parseFloat(outer_square.attr("y")) + OUTER_LEN_CHANGE;
        let length = outer_square.width() - OUTER_LEN_CHANGE*2;
        if (length < (OUTER_RESET_SIZE+2)*2) stage += 1;

        outer_square.attr("x", x);
        outer_square.attr("y", y);
        outer_square.width(length);
        outer_square.height(length);
    }

    function shrink_OC() {
        let radius = outer_circle.attr("r") - OUTER_LEN_CHANGE;
        if (radius < OUTER_RESET_SIZE+2) stage += 1;

        outer_circle.attr("r", radius);
    }

    function next_frame() {
        if (stage > 5) stage = 0;

        switch (stage) {
            case 0:
                empty(circle);
                hide(circle);
                hide(outer_circle);
                show(square);
                reset_OS();
                break;
            case 1:
                shrink_OS();
                break;
            case 2:
                fill(square);
                wait(target_delay_frames);
                break;
            case 3:
                empty(square);
                hide(square);
                hide(outer_square);
                show(circle)
                reset_OC();
                break;
            case 4:
                shrink_OC();
                break;
            case 5:
                fill(circle);
                wait(target_delay_frames);
                break;
            default:
                console.log("ERROR: Stage switch-case out of scope.");
                break;
        }
    }

    setInterval(next_frame, frames_per_ms);
});