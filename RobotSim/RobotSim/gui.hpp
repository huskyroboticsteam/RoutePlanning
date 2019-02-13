//
//  gui.hpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 1/29/19.
//  Copyright © 2019 Husky Robotics. All rights reserved.
//

#ifndef gui_hpp
#define gui_hpp

#include <stdio.h>
#include "SFML/Graphics.hpp"
#include "guielement.hpp"
#include "agent.hpp"

class Gui : public sf::Transformable, public sf::Drawable
{
public:
    Gui(unsigned int leftOffset, unsigned int topOffset, unsigned int scale, sf::Font* font);
    
    void addElement(GuiElement e, unsigned int row, unsigned int col);
    
    void addSettingToggle(std::string text, int code, int initValue, unsigned int row);
    
    void addLiveInfoPanel(std::string label, int id, Agent* agent, unsigned int row);
    
    int getEntry(const sf::Vector2f mousePos);
    
    std::vector<GuiElement> elements;
    
private:
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const;
    
    unsigned int leftOffset;
    unsigned int topOffset;
    unsigned int scale;
    
    unsigned int paddingBottom;
    unsigned int paddingRight;
    
    sf::Font* font;
};

#endif /* gui_hpp */
