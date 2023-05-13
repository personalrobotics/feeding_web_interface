// React Imports
import React, { useState } from 'react'
import Button from 'react-bootstrap/Button'
import Row from 'react-bootstrap/Row'
import { scaleWidthHeightToWindow } from '../../../helpers'
import PropTypes from 'prop-types'

import '../Button.css'

const ImageWithPointMask = (props) => {
  const width = scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0).width
  const height = scaleWidthHeightToWindow(props.imgWidth, props.imgHeight, 0, 0, 0, 0).height
  const [foodMasksToDisplay, setFoodMasksToDisplay] = useState([])

  const imageClicked = (event) => {
    let rect = event.target.getBoundingClientRect()
    let x = event.clientX - rect.left
    let y = event.clientY - rect.top
    let masks = []
    if (y < height / 2 - 15) {
      // Salad
      masks.push(
        <img
          key={1}
          style={{ width: width / 3 - 2, border: 'solid', margin: '1px' }}
          className='RectButtonsOnImg'
          src={require('./images/salad_mask1.jpg')}
        />
      )
      masks.push(
        <img
          key={2}
          style={{ width: width / 3 - 2, border: 'solid', margin: '1px' }}
          className='RectButtonsOnImg'
          src={require('./images/salad_mask2.jpg')}
        />
      )
      masks.push(
        <img
          key={3}
          style={{ width: width / 3 - 2, border: 'solid', margin: '1px' }}
          className='RectButtonsOnImg'
          src={require('./images/salad_mask3.png')}
        />
      )
    } else {
      if (x < width / 2 + 35) {
        // Chicken
        masks.push(
          <img
            key={1}
            style={{ width: width / 3 - 2, border: 'solid', margin: '1px' }}
            className='RectButtonsOnImg'
            src={require('./images/chicken_mask1.jpg')}
          />
        )
        masks.push(
          <img
            key={2}
            style={{ width: width / 3 - 2, border: 'solid', margin: '1px' }}
            className='RectButtonsOnImg'
            src={require('./images/chicken_mask2.jpg')}
          />
        )
        masks.push(
          <img
            key={3}
            style={{ width: width / 3 - 2, border: 'solid', margin: '1px' }}
            className='RectButtonsOnImg'
            src={require('./images/chicken_mask3.jpg')}
          />
        )
      } else {
        // fries
        masks.push(
          <img
            key={1}
            style={{ width: width / 3 - 2, border: 'solid', margin: '1px' }}
            className='RectButtonsOnImg'
            src={require('./images/fries_mask1.jpg')}
          />
        )
        masks.push(
          <img
            key={2}
            style={{ width: width / 3 - 2, border: 'solid', margin: '1px' }}
            className='RectButtonsOnImg'
            src={require('./images/fries_mask2.jpg')}
          />
        )
        masks.push(
          <img
            key={3}
            style={{ width: width / 3 - 2, border: 'solid', margin: '1px' }}
            className='RectButtonsOnImg'
            src={require('./images/fries_mask3.jpg')}
          />
        )
      }
    }
    setFoodMasksToDisplay(masks)
  }

  return (
    <>
      <div style={{ position: 'relative', top: '0', left: '0' }}>
        <div style={{ position: 'fixed', top: '120px', left: '25px' }}>
          <h1 style={{ background: '#FFC107' }}>Option B</h1>
        </div>
        <img
          onClick={imageClicked}
          src={props.imgSrc}
          style={{ position: 'absolute', top: '0', left: '0', width: width, height: height }}
        />
      </div>
      <div style={{ position: 'relative', top: height, left: 0 }}>
        <Row>
          {foodMasksToDisplay.map((img) => {
            img
          })}
        </Row>
        {foodMasksToDisplay}
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
          <a style={{ textDecoration: 'None', color: 'white' }} href='/test_bite-selection-ui/food_name_selection'>
            Next
          </a>
        </Button>
      </div>
    </>
  )
}

ImageWithPointMask.propTypes = {
  imgSrc: PropTypes.string.isRequired,
  imgWidth: PropTypes.number.isRequired,
  imgHeight: PropTypes.number.isRequired
}

export default ImageWithPointMask
