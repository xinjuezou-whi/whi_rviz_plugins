/******************************************************************
rviz plugin for selecte point cloud in rviz and save the selected into pcd file

Features:
- mouse selection in rviz
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-04-15: Initial version
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include <rviz_common/panel.hpp>

#include <string>
#include <memory>

// forward declaration
namespace rviz_common
{
    namespace properties
    {
        class PropertyTreeWidget;
    }
    class ViewportMouseEvent;
}

namespace whi_rviz_plugins
{
    struct PointXYZI
    {
        float x, y, z, intensity;
    };

    class PcdSelectionPanel : public rviz_common::Panel 
    {
        Q_OBJECT
    public:
        PcdSelectionPanel(QWidget* Parent = nullptr);
        virtual ~PcdSelectionPanel() = default;

    public:
        void onInitialize() override;

    private:
		void initLayout();
        bool save(const std::string& File, const std::vector<PointXYZI>& Points);

    private:
        rviz_common::properties::PropertyTreeWidget * tree_widget_{ nullptr };
    };
} // end namespace whi_rviz_plugins
