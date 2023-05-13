// React Imports
import React from 'react'
import PropTypes from 'prop-types'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'
import { scaleWidthHeightToWindow } from '../../../helpers'

import '../Button.css'

const ImageWithButtonName = (props) => {
  const width = scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0).width
  const height = scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0).height

  return (
    <>
      <div>
        <div style={{ position: 'fixed', top: '120px', left: '25px' }}>
          <h1 style={{ background: '#FFC107' }}>Option C</h1>
        </div>
        <img src={props.imgSrc} style={{ width: width, height: height }} />
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
              >
                {foodName}
              </Button>
            )
          })}
        </Row>
      </div>
      <div>
        <Button
          variant='secondary'
          className='mx-1 mb-1'
          style={{
            paddingLeft: '20px',
            paddingRight: '20px',
            marginLeft: '0px',
            marginRight: '0px',
            fontSize: '25px',
            position: 'fixed',
            right: '0px',
            bottom: '0px'
          }}
          size='lg'
        >
          <a style={{ textDecoration: 'None', color: 'white' }} href='/test_bite-selection-ui/button_overlay_selection'>
            Next
          </a>
        </Button>
      </div>
    </>
  )
}

ImageWithButtonName.propTypes = {
  imgSrc: PropTypes.string.isRequired,
  imgWidth: PropTypes.number.isRequired,
  imgHeight: PropTypes.number.isRequired,
  foodItems: PropTypes.array.isRequired
}

export default ImageWithButtonName
