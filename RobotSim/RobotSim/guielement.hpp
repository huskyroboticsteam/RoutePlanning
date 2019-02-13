//
//  guibutton.hpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 2/12/19.
//  Copyright © 2019 Husky Robotics. All rights reserved.
//

#ifndef guibutton_hpp
#define guibutton_hpp

#include <stdio.h>
#include "SFML/Graphics.hpp"
#include <iostream>

class GuiElement : public sf::Transformable, public sf::Drawable
{
public:
    sf::RectangleShape shape;
    
    int actionCode;
    
    sf::Text text;
    
    GuiElement(std::string text, int actionCode, sf::RectangleShape shape, sf::Font* font);
    
    GuiElement(int type, std::string text, int actionCode, sf::Font* font);
    
    void wasPressed();
    
private:
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const;
    
    int type; // 0 is text, 1 is on/off, 2 is load/clear
    bool toggleIsOn;
};

#endif /* guibutton_hpp */
