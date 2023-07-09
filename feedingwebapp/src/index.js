import React, { useCallback, useRef } from 'react'
import ReactDOM from 'react-dom'
import './index.css'
import App from './App'
import reportWebVitals from './reportWebVitals'
import { View } from 'react-native'
import { useWindowSize } from './helpers'

const AppComponent = () => {
  // set initial value of window height
  let windowHeight = useRef(0)
  // set a variable for storing window inner height
  let newHeight = window.innerHeight
  // callback function when height is resized
  const resizeHeight = useCallback(() => {
    windowHeight.current = newHeight
  }, [newHeight])
  // update window size when orienttaion changes with callback
  useWindowSize(resizeHeight)

  return (
    <>
      <View style={{ flex: 1, height: windowHeight.current }}>
        <App />
      </View>
    </>
  )
}

ReactDOM.render(
  <React.StrictMode>
    <AppComponent />
  </React.StrictMode>,
  document.getElementById('root')
)

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
reportWebVitals()
