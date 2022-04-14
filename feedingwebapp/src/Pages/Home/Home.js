import React from "react";
import Button from 'react-bootstrap/Button';
import Row from 'react-bootstrap/Row';

import Footer from "../Footer/Footer";

const food = ["Apple", "Banana", "Carrot", "Cucumber", "Lettuce", "Mango"];

const Home = () => {
    
    return (
        <div>
            <h1 className="text-center">Home</h1>
            <Row xs={3} md={2} className="justify-content-center mx-1 my-2">
                {food.map((value, i) => (
                    <Button variant="primary" className="mx-2 mb-2" size="lg">{value}</Button>
                ))}
            </Row>
            <Footer />
        </div>
    );
}

export default Home;