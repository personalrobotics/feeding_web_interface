// React Imports
import React, { useEffect, useRef, useState } from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'
import { scaleWidthHeightToWindow } from '../../../helpers'

import '../Button.css'

const ImageWithButtonName = (props) => {
    const imgRef = useRef()
    const [width, setWidth] = useState(scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0).width)
    const [height, setHeight] = useState(scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0).height)
    const [selectedFood, setSelectedFood] = useState("-None-")

    useEffect(() => {
        let scaledWHS = scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0)
        setWidth(scaledWHS.width)
        setHeight(scaledWHS.height)
    })

    return (
        <>
            <div>
                <img ref={imgRef} src={props.imgSrc} style={{ width: width, height: height }} />
                {console.log(props.foodItems)}
                <Row xs={3} s={2} md={3} lg={4} className='justify-content-center mx-auto my-2'>
                    {props.foodItems.map((foodName, i) => {
                        return (
                            <Button
                                key={i}
                                variant='primary'
                                className='mx-1 mb-1'
                                style={{ paddingLeft: '0px', paddingRight: '0px', marginLeft: '0px', marginRight: '0px', fontSize: '25px' }}
                                size='lg'
                                onClick={() => setSelectedFood(foodName)}
                            >{foodName}</Button>
                        )
                    })}
                </Row>
            </div>
            <div>
                <h2>Your selected Food: {selectedFood}</h2>
            </div>
        </>
    )
}

export default ImageWithButtonName
