//
//  obstacle.cpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 12/4/18.
//  Copyright © 2018 Husky Robotics. All rights reserved.
//

#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <iostream>

class Obstacle : public sf::Drawable {
public:
    Obstacle(float p1x, float p1y, float p2x, float p2y, float mapScale, bool dispQ = false) {
        // disabled drawing quadrants because I'm not using quadrants unless I have to later
        drawQuadrants = false; //dispQ;
        
        scale = mapScale;
        
        x1 = p1x;
        y1 = p1y;
        x2 = p2x;
        y2 = p2y;
        
        line.setPrimitiveType(sf::Lines);
        line.resize(2);
        line[0] = sf::Vertex(sf::Vector2f((x1 + 1) * scale, (y1 + 1) * scale));
        line[1] = sf::Vertex(sf::Vector2f((x2 + 1) * scale, (y2 + 1) * scale));
        line[0].color = sf::Color::Red;
        line[1].color = sf::Color::Red;
    }
    
    float x1;
    float y1;
    float x2;
    float y2;
    
    std::vector<std::pair<int, int>> quadrants;
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
        if (drawQuadrants)
            target.draw(quadrantFill, states);
        target.draw(line, states);
    }
    
    bool drawQuadrants;
    sf::VertexArray quadrantFill;
    sf::VertexArray line;
    
    float scale;
};
