
#include "BirdView.h"

BirdView::BirdView(cv::Mat &car_model)
{
    this->car_model = car_model;
    
    this->final_image = cv::Mat(params.total_h, params.total_w, car_model.type(), cv::Scalar(0, 0, 0)); // Data Type Must Be Same with.
}

void BirdView::add_4frames ( cv::Mat& front_image, cv::Mat& back_image, cv::Mat& left_image, cv::Mat& right_image )
{
    this->front_image = front_image;
    this->back_image = back_image;
    this->left_image = left_image;
    this->right_image = right_image;
}

cv::Mat BirdView::FI()
{
    return front_image(cv::Range::all(), cv::Range(0, params.xl));
}

cv::Mat BirdView::FM()
{
    return front_image(cv::Range::all(), cv::Range(params.xl, params.xr));
}

cv::Mat BirdView::FII()
{
    return front_image(cv::Range::all(), cv::Range(params.xr, front_image.cols));
}

cv::Mat BirdView::BIII()
{
    return back_image(cv::Range::all(), cv::Range(0, params.xl));
}

cv::Mat BirdView::BM()
{
    return back_image(cv::Range::all(), cv::Range(params.xl, params.xr));
}


cv::Mat BirdView::BIV()
{
    return back_image(cv::Range::all(), cv::Range(params.xr, back_image.cols));
}

cv::Mat BirdView::LI()
{
    return left_image(cv::Range(0, params.yt), cv::Range::all());
}

cv::Mat BirdView::LM()
{
    return left_image(cv::Range(params.yt, params.yb), cv::Range::all());
}

cv::Mat BirdView::LIII()
{
    return left_image(cv::Range(params.yb, left_image.rows), cv::Range::all());
}

cv::Mat BirdView::RII()
{
    return right_image(cv::Range(0, params.yt), cv::Range::all());
}

cv::Mat BirdView::RM()
{
    return right_image(cv::Range(params.yt, params.yb), cv::Range::all());
}

cv::Mat BirdView::RIV()
{
    return right_image(cv::Range(params.yb, right_image.rows), cv::Range::all());
}

cv::Mat BirdView::stitch_all_parts()
{
    FM().copyTo(final_image(cv::Range(0, params.yt), cv::Range(params.xl, params.xr)));
    BM().copyTo(final_image(cv::Range(params.yb, final_image.rows), cv::Range(params.xl, params.xr)));
    LM().copyTo(final_image(cv::Range(params.yt, params.yb), cv::Range(0, params.xl)));
    RM().copyTo(final_image(cv::Range(params.yt, params.yb), cv::Range(params.xr, final_image.cols)));
    
    cv::resize(car_model, car_model, cv::Size(params.xr - params.xl, params.yb - params.yt));
    car_model.copyTo(final_image(cv::Range(params.yt, params.yb), cv::Range(params.xl, params.xr)));
    
    // Step2: Get Mask for FL, FR, BL and BR
    cv::Mat mask_fl, mask_fr, mask_bl, mask_br;
    cv::Mat overlap_fl, overlap_fr, overlap_bl, overlap_br;
    
    mask_fl = uv.get_weight_mask_matrix(FI(), LI(), overlap_fl);
    mask_fr = uv.get_weight_mask_matrix(FII(), RII(), overlap_fr);
    mask_bl = uv.get_weight_mask_matrix(BIII(), LIII(), overlap_bl);
    mask_br = uv.get_weight_mask_matrix(BIV(), RIV(), overlap_br);
    
    // Step3: Fusion Overlap 
    cv::Mat merge_part_i, merge_part_ii, merge_part_iii, merge_part_iv;
    merge_part_i = uv.merge(FI(), LI(), mask_fl);
    merge_part_ii = uv.merge(FII(), RII(), mask_fr);
    merge_part_iii = uv.merge(BIII(), LIII(), mask_bl);
    merge_part_iv = uv.merge(BIV(), RIV(), mask_br);
    
    // Step4: Copy To Final Image 
    merge_part_i.copyTo(final_image(cv::Range(0, params.yt), cv::Range(0, params.xl)));
    merge_part_ii.copyTo(final_image(cv::Range(0, params.yt), cv::Range(params.xr, final_image.cols)));
    merge_part_iii.copyTo(final_image(cv::Range(params.yb, final_image.rows), cv::Range(0, params.xl)));
    merge_part_iv.copyTo(final_image(cv::Range(params.yb, final_image.rows), cv::Range(params.xr, final_image.cols)));
    
    return this->final_image;
}


