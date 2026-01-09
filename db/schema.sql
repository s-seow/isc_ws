--- Creates the iSC MySQL schema required by this project. ---

-- Create DB (if needed)
CREATE DATABASE IF NOT EXISTS isc_db
  DEFAULT CHARACTER SET utf8mb4
  DEFAULT COLLATE utf8mb4_unicode_ci;

USE isc_db;

-- Merchant master table
CREATE TABLE IF NOT EXISTS MerchantData (
    MerchantCode   VARCHAR(20)  NOT NULL,
    MerchantName   VARCHAR(255) NOT NULL,
    PRIMARY KEY (MerchantCode)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- Order table
CREATE TABLE IF NOT EXISTS iSCOrderData (
    OrderID         VARCHAR(255)  NOT NULL,
    OrderDateTime   DATETIME      NOT NULL,
    FlightDateTime  DATETIME      NOT NULL,
    MerchantCode    VARCHAR(20)   NOT NULL,
    TerminalCode    INT           NOT NULL,
    ItemQty         INT           NOT NULL,
    ItemDescription VARCHAR(255)  NOT NULL,
    IsComplete      TINYINT       NOT NULL DEFAULT 0,

    PRIMARY KEY (OrderID),
    CONSTRAINT fk_iSCOrder_Merchant
        FOREIGN KEY (MerchantCode)
        REFERENCES MerchantData (MerchantCode)
        ON UPDATE CASCADE
        ON DELETE RESTRICT
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- Item table
CREATE TABLE IF NOT EXISTS ItemDictionary (
    ItemID          BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
    MerchantCode    VARCHAR(20)      NOT NULL,
    ItemDescription VARCHAR(255)     NOT NULL,
    ItemWidthMM     INT              NOT NULL,
    ItemLengthMM    INT              NOT NULL,
    ItemHeightMM    INT              NOT NULL,

    PRIMARY KEY (ItemID),
    CONSTRAINT fk_ItemDict_Merchant
        FOREIGN KEY (MerchantCode)
        REFERENCES MerchantData (MerchantCode)
        ON UPDATE CASCADE
        ON DELETE RESTRICT,

    UNIQUE KEY uq_merchant_itemdesc (MerchantCode, ItemDescription)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;
