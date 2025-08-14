#include "Integrated.hpp"
#include "Commondefine.hpp"
#include "ICore.hpp"

namespace Manager
{
    template<typename T>
    class Storage
    {
    private:
        std::atomic<bool>       isEmpty_;
        T                       storage_;
        std::mutex              mtx_;

    public:
        Storage(T& t) :isEmpty_(true),storage_(t){}
        Storage() :isEmpty_(true){}
        ~Storage() = default;

        void setStorage(T& t)
        {
            std::lock_guard lock(mtx_);
            storage_ = std::move(t);
            isEmpty_.store(false);
        }

        const T& getStorage()
        {
            std::lock_guard lock(mtx_);
            
            isEmpty_.store(true);

            return storage_;
        }

        T snapshot() const
        {
            return storage_;
        }

        bool empty()const { return isEmpty_.load();}

        Storage* operator=(Storage& rhs)
        {
            if (this != &rhs)
            {
                this->isEmpty_ = rhs.isEmpty_;
                this->storage_ = std::move(rhs.storage_);
            }
            return this;
        }
    };

    using shoesStorage = Storage<Commondefine::shoesproperty>;
    
    class StorageManager
    {
    private:
        Log::Logger::s_ptr                                 log_;
        Integrated::w_ptr<core::ICore>                     icore_;
        std::atomic<bool>                                  criticalSection_;//true 일때 사용 가능 , false 일때 사용 불가
        std::atomic<bool>                                  workOnlyOnce_; //true 일때 사용 가능 , false 일때 사용 불가
        std::condition_variable                            workOnlyOnce_cv_;
        std::mutex                                         workOnlyOnce_mtx_;

        std::queue<Commondefine::StorageRequest>           storageRequest_;
        Commondefine::StorageRequest                       currentRequest_;

        std::mutex                                         Request_mtx_;
        std::condition_variable                            Request_cv_;
        
        std::map<int, shoesStorage>                        shelf_info_;
        std::mutex                                         storage_mtx_;
        
        std::map<int, shoesStorage>                        buffer_info_;
        std::mutex                                         buffer_info_mtx_;

    public:
        using u_ptr = Integrated::u_ptr<StorageManager>;
        using s_ptr = Integrated::s_ptr<StorageManager>;

        StorageManager(Integrated::w_ptr<core::ICore> icore, Log::Logger::s_ptr log);
        ~StorageManager();

        void SetWorkOnlyOnce(bool flag);

        bool getWorkOnlyOnce(){return workOnlyOnce_.load();}

        bool waitWorkOnlyOnce(std::chrono::milliseconds ms);

        void InitContainer();

        void setCriticalSection(bool flag);
        
        bool getCriticalSection() { return criticalSection_.load(); }
        
        bool waitCriticalSection(std::chrono::milliseconds ms);

        bool StorageRequest(Commondefine::StorageRequest storage);

        bool popRequest(Commondefine::StorageRequest& req);

        bool checkCriticalSection();

        bool resolveRequest();

        ///////////////////////////////////////////////////////////////

        bool setStorage(Commondefine::ContainerType container, Commondefine::StorageRequest shoes);

        bool getStorage(Commondefine::ContainerType container, Commondefine::StorageRequest& shoes);

        int findStorage(Commondefine::ContainerType container, Commondefine::shoesproperty& shoes);

        int findEmptyStorage(Commondefine::ContainerType container);
    };
};