#include "agent.hpp"

Agent::Agent(unsigned int mapScale, float startX, float startY, float startR, float tSpeed, float rSpeed)
{
    // internal position stored in meters
    xPos = startX;
    yPos = startY;
    // internal rotation stored in degrees
    rotation = startR;
    // internal speed that is currently unused
    transSpeed = tSpeed;
    rotSpeed = rSpeed;

    // the map's scale (pixels per meter)
    scale = mapScale;
    float fScale = (float)scale;

    // default agent size in pixels (meters * pixels per meter)
    float agentWidth = 1.f * scale;
    float agentLength = 1.5f * scale;

    // this is a rectangle
    shapeBase = sf::RectangleShape(sf::Vector2f(agentWidth, agentLength));
    shapeBase.setOrigin(agentWidth / 2.f, agentLength / 2.f);
    shapeBase.setFillColor(BASE_COLOR);
    shapeBase.setPosition((xPos + 1) * scale, (yPos + 1) * scale);
    shapeBase.setRotation(90 + startR);

    // this is a triangle, drawn on top of the rectangle
    shapeTop = sf::CircleShape(agentWidth / 2.f, 3);
    shapeTop.setOrigin(agentWidth / 2.f, agentWidth / 2.f);
    shapeTop.setFillColor(TOP_COLOR);
    shapeTop.setPosition((xPos + 1) * scale, (yPos + 1) * scale);
    shapeTop.setRotation(90 + startR);

    // the hitbox is stored as four vectors in polar coordinates
    // each vector corresponds to the displacement of one vertex from the center of the agent
    hitBox[0] = {.9f, atan2(.75f, -.5f)};
    hitBox[1] = {.9f, atan2(.75f, .5f)};
    hitBox[2] = {.9f, atan2(-.75f, .5f)};
    hitBox[3] = {.9f, atan2(-.75f, -.5f)};

    // draws the path the agent has taken
    path.setPrimitiveType(sf::Lines);
}

// simply translates the agent by a certain x and y, regardless of orientation
// adds the movement to the drawn path
void Agent::move(float dx, float dy)
{
    xPos += dx;
    yPos += dy;

    path.append(shapeBase.getPosition());

    shapeBase.move(dx * scale, dy * scale);
    shapeTop.move(dx * scale, dy * scale);

    path.append(shapeBase.getPosition());

    path[path.getVertexCount() - 2].color = PATH_COLOR;
    path[path.getVertexCount() - 1].color = PATH_COLOR;
}

// simply rotates the agent by a certain angle
void Agent::rotate(float dr)
{
    rotation += dr;
    
    if (rotation > 360)
        rotation -= 360;
    else if (rotation < 0)
        rotation += 360;

    shapeBase.rotate(dr);
    shapeTop.rotate(dr);
}

// erases the path drawn so far
void Agent::clearPath() { path.clear(); }

void Agent::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    states.transform *= getTransform();

    target.draw(path, states);
    target.draw(shapeBase, states);
    target.draw(shapeTop, states);
}
