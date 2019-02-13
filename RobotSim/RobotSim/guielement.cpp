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

GuiElement::GuiElement(int type, std::string newText, int actionCode, sf::Font* font) {
    this->actionCode = actionCode;
    
    text = sf::Text();
    text.setFont(*font);
    text.setCharacterSize(24); // in pixels
    text.setFillColor(sf::Color::Black);
    
    switch (type) {
        case 1 : {
            // off button
            shape = sf::RectangleShape(sf::Vector2f(50, 50));
            shape.setFillColor(sf::Color::Red);
            
            text.setString("Off");
            this->type = 1;
            toggleIsOn = false;
            break;
        }
        case 2 : {
            // on button
            shape = sf::RectangleShape(sf::Vector2f(50, 50));
            shape.setFillColor(sf::Color::Green);
            
            text.setString("On");
            this->type = 1;
            toggleIsOn = true;
            break;
        }
        case 3 : {
            // load button
            shape = sf::RectangleShape(sf::Vector2f(200, 50));
            shape.setFillColor(sf::Color(200, 200, 200));
            
            text.setString("Load obstacles.txt");
            this->type = 2;
            toggleIsOn = false;
            break;
        }
        default : {
            // text box
            shape = sf::RectangleShape(sf::Vector2f(200, 50));
            shape.setFillColor(sf::Color(200, 200, 200));
            
            text.setString(newText);
            break;
        }
    }
}

void GuiElement::wasPressed() {
    switch (type) {
        case 1 : {
            if (toggleIsOn) {
                shape.setFillColor(sf::Color::Red);
                text.setString("Off");
            }
            else {
                shape.setFillColor(sf::Color::Green);
                text.setString("On");
            }
            toggleIsOn = !toggleIsOn;
            break;
        }
        case 2 : {
            if (toggleIsOn) {
                text.setString("Load");
            }
            else {
                text.setString("Clear All");
            }
            toggleIsOn = !toggleIsOn;
            break;
        }
    }
}

void GuiElement::draw(sf::RenderTarget &target, sf::RenderStates states) const {
    states.transform *= getTransform();
    target.draw(shape, states);
    target.draw(text, states);
}
