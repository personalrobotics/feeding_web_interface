/**
 * Takes in an imageWidth and imageHeight, and returns a width and height that
 * maintains the same aspect ration but fits within the window.
 *
 * TODO (amaln): Clean up and add comments to the code, understand why there is a -70.
 */
export function scaleWidthHeightToWindow(imageWidth, imageHeight) {
  let imageAspectRatio = imageWidth / imageHeight
  let windowWidth = window.innerWidth
  let windowHeight = window.innerHeight
  let returnWidth = 0
  let returnHeight = 0
  if (windowWidth / windowHeight > imageAspectRatio) {
    returnHeight = windowHeight - 70
    returnWidth = imageAspectRatio * returnHeight
  } else {
    returnWidth = windowWidth
    returnHeight = (1 / imageAspectRatio) * returnWidth
  }
  return {
    width: returnWidth,
    height: returnHeight
  }
}
