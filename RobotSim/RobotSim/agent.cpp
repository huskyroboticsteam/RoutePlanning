//
//  agent.cpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 11/13/18.
//  Copyright © 2018 Husky Robotics. All rights reserved.
//

#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <iostream>

#define PI 3.141592654

class Agent : public sf::Drawable, public sf::Transformable {
    
public:
    Agent(sf::Vector2f startPos) {
        xPos = 0.f;
        yPos = 0.f;
        rotation = 0.f;
        
        shape = sf::CircleShape(SHAPE_RADIUS, 3); // equilateral triangle
        shape.setOrigin(SHAPE_RADIUS, SHAPE_RADIUS);
        shape.setFillColor(SHAPE_COLOR);
        shape.setPosition(startPos);
        
        path.setPrimitiveType(sf::LineStrip);
        path.resize(99999); // arbitrary max path length
        path[0] = startPos;
        
        pathIndex = 1;
    }
    
    float getX() { return xPos; }
    float getY() { return yPos; }
    float getRotation() { return rotation; }
    
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
        states.transform *= getTransform();
        
        target.draw(path, states);
        target.draw(shape, states);
    }
    
    sf::CircleShape shape;
    sf::VertexArray path;
    
    int pathIndex;
    
    float xPos;
    float yPos;
    float rotation;
    
    const float SHAPE_RADIUS = 20.f;
    const sf::Color SHAPE_COLOR = sf::Color::Green;
    const sf::Color PATH_COLOR = sf::Color::Green;
};
