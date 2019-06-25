#include "DummyMassBodySanitizer.h"


namespace VirtualRobot::mujoco
{

struct DummyMassVisitor : public mjcf::Visitor
{
    using mjcf::Visitor::Visitor;
    virtual bool visitEnter(const mjcf::AnyElement& element) override;
};

bool DummyMassVisitor::visitEnter(const mjcf::AnyElement& element)
{
    if (element.is<mjcf::Body>())
    {
        mjcf::Body body = element.as<mjcf::Body>();
        if (!body.hasMass())
        {
            body.addDummyInertial();
        }
    }
    return true;
}



DummyMassBodySanitizer::DummyMassBodySanitizer()
{}

void DummyMassBodySanitizer::sanitize(mjcf::Document& document, mjcf::Body root)
{
    DummyMassVisitor visitor(document);
    root.accept(visitor);
}





}
