//
//  guibutton.cpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 2/12/19.
//  Copyright © 2019 Husky Robotics. All rights reserved.
//

#include "guielement.hpp"

GuiElement::GuiElement(std::string newText, int pressedCode, sf::RectangleShape shape, sf::Font* font) {
    this->actionCode = pressedCode;
    this->shape = shape;
    
    text = sf::Text();
    text.setFont(*font);
    text.setString(newText);
    text.setCharacterSize(24); // in pixels
    text.setFillColor(sf::Color::Black);
}

GuiElement::GuiElement(std::string newText, int pressedCode, int shapeCode, sf::Font* font) {
    this->actionCode = pressedCode;
    
    text = sf::Text();
    text.setFont(*font);
    text.setString(newText);
    text.setCharacterSize(24); // in pixels
    text.setFillColor(sf::Color::Black);
    
    switch (shapeCode) {
        case 1 : {
            // on button
            shape = sf::RectangleShape(sf::Vector2f(50, 50));
            shape.setFillColor(sf::Color::Green);
            break;
        }
        case 2 : {
            // off button
            shape = sf::RectangleShape(sf::Vector2f(50, 50));
            shape.setFillColor(sf::Color::Red);
            break;
        }
        default : {
            // text box
            shape = sf::RectangleShape(sf::Vector2f(200, 50));
            shape.setFillColor(sf::Color(200, 200, 200));
            break;
        }
    }
}

void GuiElement::wasPressed() {
    
}

void GuiElement::draw(sf::RenderTarget &target, sf::RenderStates states) const {
    states.transform *= getTransform();
    target.draw(shape, states);
    target.draw(text, states);
}
