#include "environment.hpp"




bool test_add_marble_to_map(environment& DUT, const cv::Point& marble, int expected_ans)
{
    return DUT.add_marble(marble) == expected_ans;
}


bool test_pick_up_marble(cv::Mat& img)
{
    environment test(0.1, -0.5, 5.0, img);
    cv::Point point{5,2};
    std::cout << img << std::endl;
    test.print_environment_type_map();
    test.add_marble({10,10});
    test.add_marble({12,10});
    test.add_marble({14,10});
    test.print_environment_type_map();
    test.init_environment();

    auto remaining_marbles = test.get_current_remaining_marbles();
    auto current_environment = test.get_environment(remaining_marbles);

    test.pick_up_marble({10,10});
    test.print_environment_type_map();
    auto remaining_marbles_new = test.get_current_remaining_marbles();

    assert(remaining_marbles != remaining_marbles_new);
    test.pick_up_marble({11,10});
    return true;

}


bool run_test()
{
    cv::Mat test (7,7, CV_8UC1 , cv::Scalar(0));
    environment DUT(0.1, -0.5, 5.0, test);
    bool test_passed = false;

    test_passed = test_add_marble_to_map(DUT, {2,0}, 2);
    std::cout << "Running test: test_add_marble_to_map(DUT, {2,0}, 2)"  << std::boolalpha <<  "    TEST: " << test_passed << std::endl;
    assert(test_passed == true);

    test_passed = test_add_marble_to_map(DUT, {5,0}, 5);
    std::cout << "Running test: test_add_marble_to_map(DUT, {5,0}, 5)"  << std::boolalpha <<  "    TEST: " << test_passed << std::endl;
    assert(test_passed == true);


    test_passed = test_add_marble_to_map(DUT, {2,1}, 9);
    std::cout << "Running test: test_add_marble_to_map(DUT, {2,1}, 9)"  << std::boolalpha <<  "    TEST: " << test_passed << std::endl;
    assert(test_passed == true);


    test_passed = test_add_marble_to_map(DUT, {0,1}, 7);
    std::cout << "Running test: test_add_marble_to_map(DUT, {0,1}, 7)"  << std::boolalpha <<  "    TEST: " << test_passed << std::endl;
    assert(test_passed == true);


    test_passed = test_add_marble_to_map(DUT, {6,6}, 48);
    std::cout << "Running test: test_add_marble_to_map(DUT, {6,6}, 48)"  << std::boolalpha <<  "    TEST: " << test_passed << std::endl;
    assert(test_passed == true);

    cv::Mat src, src_gray, binary;
    src = cv::imread("export.png");
    cv::cvtColor(src,src_gray, CV_BGR2GRAY);
    cv::threshold(src_gray, binary, 250, 255, CV_THRESH_BINARY);

    test_pick_up_marble(binary);
    //cv::imshow("Environment", binary);
    //cv::waitKey();

    return true;
}
