# Methodology for Testing Responsiveness 

After developing an app feature, make sure to test it on the following setups. Total 4 devices as listed below.
- **Smartphone:** iPhone 14 Plus [Tyler]; Width: 428 & Height: 926
- **Tablet:** iPad Pro (6th gen 12.9") [Tyler]; Width: 1024 & Height: 1366
- **External Monitor:** Width: 4096 & Height: 2304
- **Laptop:** Width: 2560 & Height: 1600
    - Full-screen 
    - Half-screen (vertical split)
    - Quarter screen 

For the smartphone and laptops, test on the 4 browsers specified below. For the tablet and monitor, pick one browser to test it on. 

In total, you will run 18 tests  (4 browsers * 1 smartphone + 1 tablet + 4 browsers * 3 laptops + 1 monitor). 

If you don’t have the listed devices available for testing, please “Add custom device” in your browser’s responsive design mode using the width and height of the specific device’s resolution below. Please find the appropriate link in the [Resources](#resources) section at the bottom of this page for each browser's documentation on how to do this *except* Safari. 

## Browsers:  
Chrome, Firefox, Edge, Safari

## Screen Orientation (for smartphone and tablet only):  
Landscape and portrait 

In portrait, pages should have their content appear on one screen and not require scrolling. In landscape, scrolling may be unavoidable for some screens, but try to rearrange the content horizontally in order to minimize required scrolling. 

## Resources:
1. https://www.ios-resolution.com/
2. https://support.apple.com/kb/SP748?locale=en_US
3. Adding custom device in Chrome: https://developer.chrome.com/blog/add-a-new-custom-device-as-a-preset/
4. Adding custom device in Edge: https://learn.microsoft.com/en-us/microsoft-edge/devtools-guide-chromium/device-mode/
5. Adding custom device in Firefox:
https://firefox-source-docs.mozilla.org/devtools-user/responsive_design_mode/#creating-custom-devices



