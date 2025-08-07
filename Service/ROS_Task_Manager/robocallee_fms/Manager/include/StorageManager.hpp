#include "Integrated.hpp"
#include "Commondefine.hpp"
#include "ICore.hpp"

namespace Manager
{
    class StorageManager
    {
    private:
        Log::Logger::s_ptr                                 log_;
        Integrated::w_ptr<core::ICore>                     icore_;
        std::atomic<bool>                                  pickUpPlace_; //true 일때 사용 가능 , false 일때 사용 불가
        std::queue<Commondefine::StorageRequest>           storageQueue_;

        std::mutex                                         storage_mtx_;
    public:
        StorageManager(Integrated::w_ptr<core::ICore> icore, Log::Logger::s_ptr log);
        ~StorageManager();


        bool Enqueue(Commondefine::StorageRequest storage);
    };
};