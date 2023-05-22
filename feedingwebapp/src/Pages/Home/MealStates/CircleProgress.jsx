import { Circle } from './progressbar.js'
import React, { useEffect } from 'react'

export default function CircleProgress() {
  return (
    <>
      <CreateCircle />
      <div id='container'></div>
    </>
  )
}

function CreateCircle() {
  useEffect(() => {
    var bar = new Circle('#container', { easing: 'easeInOut' })
    bar.animate(1)
  }, [])
}
