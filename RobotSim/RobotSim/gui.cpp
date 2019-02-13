//
//  gui.cpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 1/29/19.
//  Copyright © 2019 Husky Robotics. All rights reserved.
//

#include "gui.hpp"

Gui::Gui(unsigned int leftOffset, unsigned int topOffset, unsigned int scale, sf::Font* font) {
    this->leftOffset = leftOffset;
    this->topOffset = topOffset;
    this->scale = scale;
    
    this->font = font;
    
    this->move(leftOffset, topOffset);
    
    paddingBottom = 10;
    paddingRight = 10;
}

void Gui::addElement(GuiElement e, unsigned int row, unsigned int col) {
    e.move(col * (200 + paddingRight), row * (50 + paddingBottom));
    elements.push_back(e);
}

void Gui::addSettingToggle(std::string text, int code, unsigned int row) {
    GuiElement label(text, -1, 0, font);
    label.move(0, row * (50 + paddingBottom));
    
    // todo determine setting from setting code
    
    GuiElement button("On", code, 1, font);
    button.move(200 + paddingRight, row * (50 + paddingBottom));
    
    elements.push_back(label);
    elements.push_back(button);
}

// returns the action code of the button the mouse is over
int Gui::getEntry(sf::Vector2f mousePos) {
    for(int i = 0; i < elements.size(); i++)
    {
        /* Translate point to use the entry's local coordinates. */
        sf::Vector2f point = mousePos;
        
        point -= elements[i].getPosition(); // local position of element within gui
        point -= this->getPosition(); // local position of gui within window
        
        sf::Vector2f buttonSize = elements[i].shape.getSize();
        
        if (point.x > 0 && point.x < buttonSize.x && point.y > 0 && point.y < buttonSize.y) {
            elements[i].wasPressed();
            return elements[i].actionCode;
        }
    }
    
    return -1;
}

void Gui::draw(sf::RenderTarget &target, sf::RenderStates states) const {
    states.transform *= getTransform();
    
    for (GuiElement e : elements) {
        target.draw(e, states);
    }
}
