#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace graph_search_contact_planner_sample{
  void sample0_display();
  class sample0_displayItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample0_displayItem>("sample0_displayItem"); }
  protected:
    virtual void main() override{ sample0_display(); return;}
  };
  typedef cnoid::ref_ptr<sample0_displayItem> sample0_displayItemPtr;

  void sample1_box();
  class sample1_boxItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_boxItem>("sample1_boxItem"); }
  protected:
    virtual void main() override{ sample1_box(); return;}
  };
  typedef cnoid::ref_ptr<sample1_boxItem> sample1_boxItemPtr;
  class GraphSearchContactPlannerSamplePlugin : public cnoid::Plugin
  {
  public:
    GraphSearchContactPlannerSamplePlugin() : Plugin("GraphSearchContactPlannerSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample0_displayItem::initializeClass(this);
      sample1_boxItem::initializeClass(this);
      return true;
    }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(graph_search_contact_planner_sample::GraphSearchContactPlannerSamplePlugin)
