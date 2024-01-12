#!/usr/bin/env node

const { chromium } = require('playwright')
const logId = 'start_robot_browser.js'

let robotHostname = 'localhost:' + process.env.REACT_APP_PORT
if (process.argv.length > 2) {
  robotHostname = process.argv[2]
}

;(async () => {
  var num_tries = 0
  var max_tries = 50 // -1 means try forever

  ///////////////////////////////////////////////
  // sleep code from
  // https://stackoverflow.com/questions/951021/what-is-the-javascript-version-of-sleep
  function sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms))
  }
  ///////////////////////////////////////////////

  const browser = await chromium.launch({
    headless: true, // default is true
    ignoreHTTPSErrors: true, // avoid ERR_CERT_COMMON_NAME_INVALID
    defaultViewport: null,
    args: [
      '--use-fake-ui-for-media-stream', //gives permission to access the robot's cameras and microphones (cleaner and simpler than changing the user directory)
      '--disable-features=WebRtcHideLocalIpsWithMdns', // Disables mDNS hostname use in local network P2P discovery. Necessary for enterprise networks that don't forward mDNS traffic
      '--ignore-certificate-errors'
    ]
  })
  const page = await browser.newPage()

  while (num_tries < max_tries) {
    try {
      await page.goto(`http://${robotHostname}/robot`)
      console.log(logId + ': finished loading')
      break
    } catch (e) {
      console.log(logId + ': trying again')
      console.log(e)
      await sleep(200)
      num_tries += 1
    }
  }

  console.log(logId + ': start script complete')
})()
