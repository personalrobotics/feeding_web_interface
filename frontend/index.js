"use strict";

// program mode: user mode (default) / developer mode
let isUserMode = true;
// list of food
let foodImages = ["pics/apple.jpg", "pics/banana.jpg", "pics/cake.jpg", "pics/candy.jpg", "pics/ice_cream.jpg", "pics/strawberry.jpg"];

$(function() {
    $(document).ready(function() {
        // Event handlers
        // mode buttons
        $("#user_mode_btn").click(switchMode);
        $("#dev_mode_btn").click(switchMode);
        // back buttons
        $("#back_to_food_image_container").click(backToFoodContainer);
        $("#back_to_video_stream_container").click(backToVideoStream);
        // random food selection
        $("#random_pick_btn").click(chooseFoodRandomly);
        $("#random_image").click(showImage);
        // TODO: change this to event handlers for each food
        $("#video_stream").click(showAction);
        // food actions
        var foodActionButtons = document.getElementById("actions").querySelectorAll(".action_btn");
        for (var i = 0; i < foodActionButtons.length; i++) {
            console.log(foodActionButtons[i]);
            foodActionButtons[i].addEventListener("click", performAction);
            foodActionButtons[i].actionName = foodActionButtons[i].innerHTML;
        }

        // Get image from backend
        for (var i = 0; i < foodImages.length; i++) { 
            var imageDiv = document.createElement("div");
            imageDiv.className = "food_image";
            // event handler
            imageDiv.addEventListener("click", showImage);

            let imgaeName = foodImages[i].split("/")[1].split(".")[0];
            imgaeName = imgaeName.charAt(0).toUpperCase() + imgaeName.substring(1);
            var title = document.createElement("p");
            title.innerHTML = imgaeName;
            var image = document.createElement("img");
            image.src = foodImages[i];
            image.alt = imgaeName;
            image.style.width = "100px";
            image.style.height = "100px";

            imageDiv.appendChild(title);
            imageDiv.appendChild(image);
            $("#food_image_container").append(imageDiv);
        }

        // TODO: get vieo stream from backend, and insert in #video_stream
        
    });

    function switchMode() {
        if (this.innerHTML == "User Mode") {
            isUserMode = true;
            $("#user_mode_btn").css({"background-color": "#adc7dc", "color": "black"});
            $("#dev_mode_btn").css({"background-color": "#d6e3ed", "color": "white"});
        } else {  // developer mode
            isUserMode = false;
            $("#user_mode_btn").css({"background-color": "#d6e3ed", "color": "white"});
            $("#dev_mode_btn").css({"background-color": "#adc7dc", "color": "black"});
        }
    }

    function backToFoodContainer() {
        $("#food_display_container").css("display", "grid");
        $("#video_stream_container").css("display", "none");
    }

    function chooseFoodRandomly() {
        let randomImage = document.getElementById("random_image");
        let randomNum = getRandomInt(0, foodImages.length);
        randomImage.src = foodImages[randomNum];
        randomImage.alt = "random image";
    }

    function showImage() {
        // dev mode only
        if (!isUserMode && this.alt != "question mark") {
            $("#food_display_container").css("display", "none");
            $("#video_stream_container").css("display", "block");
        }
    }

    function backToVideoStream() {
        $("#video_stream_container").css("display", "block");
        $("#food_choosing_interface").css("display", "none");
    }

    function showAction() {
        $("#video_stream_container").css("display", "none");
        $("#food_choosing_interface").css("display", "block");
    }

    function performAction(event) {
        // TODO: substitute alert() with the action result sent from backend
        alert("Action: " + event.target.actionName);
    }

    function getRandomInt(min, max) {
        min = Math.ceil(min);
        max = Math.floor(max);
        return Math.floor(Math.random() * (max - min)) + min; //The maximum is exclusive and the minimum is inclusive
    }
});
