#include "eigen_conversion.h"


namespace Eigen
{
    template <>
    void from_json<Vector3f>(const nlohmann::json& j, MatrixBase<Vector3f>& vector)
    {
        if (j.is_object())
        {
            vector.x() = j.at("x").get<float>();
            vector.y() = j.at("y").get<float>();
            vector.z() = j.at("z").get<float>();
        }
        else  // j.is_array()
        {
            jsonbase::from_json(j, vector);
        }
    }


    template <>
    void from_json<Matrix4f>(const nlohmann::json& j, MatrixBase<Matrix4f>& matrix)
    {
        if (j.is_object())
        {
            const auto& j_ori = j.at("ori");
            Eigen::Matrix3f ori;

            if (j_ori.is_object())
            {
                ori = j_ori.get<Eigen::Quaternionf>().toRotationMatrix();
            }
            else
            {
                ori = j_ori.get<Eigen::Matrix3f>();
            }

            matrix = math::Helpers::Pose(j.at("pos").get<Eigen::Vector3f>(), ori);
        }
        else
        {
            jsonbase::from_json(j, matrix);
        }
    }
}
