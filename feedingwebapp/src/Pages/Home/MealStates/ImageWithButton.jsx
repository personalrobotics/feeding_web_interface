// React Imports
import React, { useEffect, useRef, useState } from 'react'
import ReactDOM from 'react-dom/client'
import { Button } from 'react-bootstrap'
import { RoundedRect } from 'react-svg-path'
import { scaleWidthHeightToWindow } from '../../../helpers'

import '../Button.css'

const ImageWithButton = (props) => {
    const imgRef = useRef()
    const [width, setWidth] = useState(scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0).width)
    const [height, setHeight] = useState(scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0).height)
    const [scaleFactor, setScaleFactor] = useState(scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0).scaleFactor)
    const [selectedFood, setSelectedFood] = useState("-None-")

    useEffect(() => {
        let scaledWHS = scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0)
        setWidth(scaledWHS.width)
        setHeight(scaledWHS.height)
        setScaleFactor(scaledWHS.scaleFactor)
    })

    // function clickRect(foodItemSelected) {
    //     console.log(foodItemSelected)
    //     setSelectedFood(foodItemSelected)
    // }

    return (
        <>
            <div style={{ position: 'relative', top: '0', left: '0' }}>
                <img ref={imgRef} src={props.imgSrc} style={{ position: 'absolute', top: '0', left: '0', width: width, height: height }} />
                {props.buttonCenters.map((location) => {
                    let xVal = location.x * scaleFactor
                    let yVal = location.y * scaleFactor
                    return (
                        <svg style={{ position: 'absolute', top: yVal, left: xVal, width: props.rectWidth * scaleFactor, height: props.rectHeight * scaleFactor, zIndex: 1 }}>
                            <svg width={width} height={height}>
                                <rect
                                    x={0}
                                    y={0}
                                    rx='5'
                                    ry='5'
                                    width={props.rectWidth * scaleFactor}
                                    height={props.rectHeight * scaleFactor}
                                    fillOpacity={0.2}
                                    stroke='black'
                                    strokeWidth={5}
                                    className='RectButtonsOnImg'
                                    onClick={() => setSelectedFood(location.name)}
                                />
                            </svg>
                        </svg>
                    )
                })}
            </div>
            <div>
                <h2 style={{ position: "relative", top: height, left: 0 }}>Your selected Food: {selectedFood}</h2>
            </div>
        </>
    )
}

export default ImageWithButton
