// React Imports
import React from 'react'
import Row from 'react-bootstrap/Row'

// Local Imports
import '../Home.css'

/**
 * The LabelGeneration component appears after the robot has moved above the plate.
 * It enables users to input labels for the food items they will be eating in the meal.
 */
const LabelGeneration = () => {
  // Font size for text
  let textFontSize = '4.2vh'
  // Margin
  let margin = '5vh'

  /** Get the full page view
   *
   * @returns {JSX.Element} the full page view
   */
  /** 
  const fullPageView = useCallback(() => {
    return (
      <>
        <Row xs={1} md={1} className='justify-content-center mx-2 my-2'>
          <p className='transitionMessage' style={{ marginBottom: margin, marginTop: '0', fontSize: textFontSize }}>
            Please label the food items you will be eating in this meal.
          </p>
        </Row>
      </>
    )
  })*/

  // Render the component
  return (
    <>
      <Row xs={1} md={1} className='justify-content-center mx-2 my-2'>
        <p className='transitionMessage' style={{ marginBottom: margin, marginTop: '0', fontSize: textFontSize }}>
          Please label the food items you will be eating in this meal.
        </p>
      </Row>
    </>
  )
}

export default LabelGeneration
