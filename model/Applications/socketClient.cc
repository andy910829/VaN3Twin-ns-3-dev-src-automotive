#include "socketClient.h"

char* socketClientFunction(const char* content) {
    // std::cout << "Socket Client Function Called with content: " << content << std::endl;
    // 1. 建立 Socket
    int sock = 0;
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return "Socket creation error";
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // 2. 將 IPv4 位址從文字轉換為二進位格式
    // 我們將連線到本地主機 (127.0.0.1)
    if (inet_pton(AF_INET, "140.124.71.80", &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        return "Invalid address/ Address not supported";
    }

    // 3. 連線到伺服器
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed" << std::endl;
        return "Connection Failed";
    }
    // std::cout << "Connected to server successfully." << std::endl;
    // 4. 傳送資料給伺服器
    send(sock, content, strlen(content), 0);
    // std::cout << "Message sent" << std::endl;

    // 5. 接收伺服器的回覆 (可選)
    char* buffer = (char *)calloc(1024,sizeof(char));

    read(sock, buffer, 1024);
    // std::cout << "Message from server: " << buffer << std::endl;

    // 6. 關閉 Socket
    close(sock);
    return buffer;
}