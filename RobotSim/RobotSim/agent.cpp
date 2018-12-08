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
#include <math.h>

#include "map.cpp"

#define PI 3.141592654

class Agent : public sf::Drawable, public sf::Transformable {
    
public:
    Agent(sf::Vector2f startPos, Map map, float startX = 0.f, float startY = 0.f, float startR = 45.f, sf::Color color = sf::Color::Green) {
        xPos = startX;
        yPos = startY;
        rotation = startR;
        
        shape = sf::CircleShape(SHAPE_RADIUS, 3); // equilateral triangle
        shape.setOrigin(SHAPE_RADIUS, SHAPE_RADIUS);
        shape.setFillColor(color);
        shape.setPosition(startPos);
        shape.setRotation(90 + startR);
        
        path.setPrimitiveType(sf::Lines);
        
        pixelsPerMeter = map.getScale();
        
        pathColor = color;
        
        internalMap = &map;
    }
    
    void move(float distance) {
        float xOffset = distance * cos(rotation * PI / 180);
        float yOffset = distance * sin(rotation * PI / 180);
        
        float newX = xPos + xOffset;
        float newY = yPos + yOffset;
        
        if (newX < 0)
            xOffset = 0 - xPos;
        if (newX > (internalMap->getWidth() - 1))
            xOffset = internalMap->getWidth() - 1 - xPos;
        if (newY < 0)
            yOffset = 0 - yPos;
        if(newY > (internalMap->getHeight() - 1))
            yOffset = internalMap->getWidth() - 1 - yPos;
        
        xPos += xOffset;
        yPos += yOffset;
        
        sf::Vertex start = shape.getPosition();
        start.color = pathColor;
        
        path.append(start);
        
        shape.move(xOffset * pixelsPerMeter, yOffset * pixelsPerMeter);
        
        sf::Vertex end = shape.getPosition();
        end.color = pathColor;
        
        path.append(end);
    }
    
    void rotate(float rotate) {
        rotation += rotate;
        shape.rotate(rotate);
    }
    
    void place(float x, float y, float rot, bool drawPath = true) {
        xPos = x;
        yPos = y;
        rotation = rot;
        
        // update shape
        shape.setRotation(90.f + rot);
        
        if (drawPath) {
            sf::Vertex start = shape.getPosition();
            start.color = pathColor;
            path.append(start);
        }
        shape.setPosition((x + 1.f) * pixelsPerMeter, (y + 1.f) * pixelsPerMeter);
        if (drawPath) {
            sf::Vertex end = shape.getPosition();
            end.color = pathColor;
            path.append(end);
        }
    }
    
    void erasePath() {
        path = sf::VertexArray();
        path.setPrimitiveType(sf::Lines);
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
    
    float xPos;
    float yPos;
    float rotation;
    
    int pixelsPerMeter;
    
    Map* internalMap;
    
    const float SHAPE_RADIUS = 20.f;
    //const sf::Color SHAPE_COLOR = sf::Color::Green;
    sf::Color pathColor;
};
