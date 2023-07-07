import React from 'react'
import ReactDOM from 'react-dom'
import './index.css'
import App from './App'
import reportWebVitals from './reportWebVitals'
import { View } from 'react-native'
import { useWindowSize } from './helpers'

const HeightComponent = () => {
  let currentWindowSize = useWindowSize()
  return (
    <>
      <View style={{ flex: 1, height: currentWindowSize.height }}>
        <App />
      </View>
    </>
  )
}

ReactDOM.render(
  <React.StrictMode>
    <HeightComponent />
  </React.StrictMode>,
  document.getElementById('root')
)

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
reportWebVitals()
