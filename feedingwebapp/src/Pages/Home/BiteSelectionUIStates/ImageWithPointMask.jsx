// React Imports
import React, { useEffect, useRef, useState } from 'react'
import { scaleWidthHeightToWindow } from '../../../helpers'

import '../Button.css'

const ImageWithPointMask = (props) => {
    const imgRef = useRef()
    const row = [0, 1, 2]
    const col = [0, 1, 2]
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

    let i = 0
    const buttonss = []
    let widthSpace = width / 3
    let heightSpace = height / 3
    while (i < height) {
        let j = 0   
        console.log(i)
        while (j < width) {
            console.log(j)
            buttonss.push(
                <svg style={{ position: 'absolute', top: i, left: j, width: widthSpace, height: heightSpace, zIndex: 1 }}>
                    <svg width={width} height={height}>
                        <rect
                            x={0}
                            y={0}
                            rx='5'
                            ry='5'
                            width={widthSpace}
                            height={heightSpace}
                            fillOpacity={0.2}
                            stroke='black'
                            strokeWidth={5}
                            className='RectButtonsOnImg'
                            onClick={() => setSelectedFood(location.name)}
                        />
                    </svg>
                </svg>
            )
            j = j + widthSpace
        }
        i += heightSpace
    }

    return (
        <>
            <div style={{ position: 'relative', top: '0', left: '0' }}>
                <img ref={imgRef} src={props.imgSrc} style={{ position: 'absolute', top: '0', left: '0', width: width, height: height }} />
                {buttonss}
            </div>
            <div>
                <h2 style={{ position: "relative", top: height, left: 0 }}>Your selected Food: {selectedFood}</h2>
            </div>
        </>
    )
}

export default ImageWithPointMask
