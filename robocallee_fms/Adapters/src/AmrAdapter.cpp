#include "AmrAdapter.hpp"

using namespace Adapter;

AmrAdapter::AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log)
    :Icore_(Icore),log_(log)
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 생성");
    auto p = Icore_.lock();
    p->SetAmrNextStep(AmrStep_num);
}

AmrAdapter::~AmrAdapter()
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 소멸");
}