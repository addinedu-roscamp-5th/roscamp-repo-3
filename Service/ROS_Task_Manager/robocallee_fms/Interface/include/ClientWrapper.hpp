#pragma once
#include "Integrated.hpp"
#include "Commondefine.hpp"

#include "rclcpp/rclcpp.hpp"

namespace CW
{
    class ClientWrapperBase
    {
    public:
        using s_ptr = Integrated::s_ptr<ClientWrapperBase>;
        
        virtual ~ClientWrapperBase() = default;

        virtual bool wait_for_service(std::chrono::seconds timeout) = 0;

        virtual std::string getName() const = 0;
    };

    template<typename ServiceT>
    class ClientWrapper : public ClientWrapperBase
    {
    public:
        ClientWrapper(const std::string& name, std::shared_ptr<rclcpp::Client<ServiceT>> client)
            : name_(name), client_(client) {}

        bool wait_for_service(std::chrono::seconds timeout) override { return client_->wait_for_service(timeout);}

        std::string getName() const override { return name_;}

        std::shared_ptr<rclcpp::Client<ServiceT>> getClient() const { return client_; }

    private:
        std::string name_;
        std::shared_ptr<rclcpp::Client<ServiceT>> client_;
    };
};