/*
* Copyright 2015-2019 Espressif Systems (Shanghai) PTE LTD
*
* Licensed to the Apache Software Foundation (ASF) under one
* or more contributor license agreements.  See the NOTICE file
* distributed with this work for additional information
* regarding copyright ownership.  The ASF licenses this file
* to you under the Apache License, Version 2.0 (the
* "License"); you may not use this file except in compliance
* with the License.  You may obtain a copy of the License at
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing,
* software distributed under the License is distributed on an
* "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
* KIND, either express or implied.  See the License for the
* specific language governing permissions and limitations
* under the License.
*/


#include "syscfg/syscfg.h"

#if MYNEWT_VAL(BLE_STORE_CONFIG_PERSIST)

#include <string.h>
#include <esp_system.h>
#include "sysinit/sysinit.h"
#include "host/ble_hs.h"
#include "store/config/ble_store_config.h"
#include "ble_store_config_priv.h"
#include "esp_log.h"
#include "nvs.h"
#include "../../../src/ble_hs_resolv_priv.h"


#define NIMBLE_NVS_STR_NAME_MAX_LEN              16
#define NIMBLE_NVS_PEER_SEC_KEY                  "peer_sec"
#define NIMBLE_NVS_OUR_SEC_KEY                   "our_sec"
#define NIMBLE_NVS_CCCD_SEC_KEY                  "cccd_sec"
#define NIMBLE_NVS_PEER_RECORDS_KEY              "p_dev_rec"
#define NIMBLE_NVS_NAMESPACE                     "nimble_bond"

static const char *TAG = "NIMBLE_NVS";

static void bluedroid_nvs_to_nimble_nvs(void);

/*****************************************************************************
 * $ MISC                                                                    *
 *****************************************************************************/

static void
get_nvs_key_string(int obj_type, int index, char *key_string)
{
    if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
        sprintf(key_string, "%s_%d", NIMBLE_NVS_PEER_RECORDS_KEY, index);
    } else {
        if (obj_type == BLE_STORE_OBJ_TYPE_PEER_SEC) {
            sprintf(key_string, "%s_%d", NIMBLE_NVS_PEER_SEC_KEY, index);
        } else if (obj_type == BLE_STORE_OBJ_TYPE_OUR_SEC) {
            sprintf(key_string, "%s_%d", NIMBLE_NVS_OUR_SEC_KEY, index);
        } else {
            sprintf(key_string, "%s_%d", NIMBLE_NVS_CCCD_SEC_KEY, index);
        }
    }
}

/* compares values at two addresses of size = item_size
* @Returns               index if entries match
*                       -1 if mismatch
*/
static int
get_nvs_matching_index(void *nvs_val, void *db_list, int db_num, size_t
                       item_size)
{
    uint8_t *db_item = (uint8_t *)db_list;
    int i;

    for (i = 0; i < db_num; i++) {
        if (memcmp(nvs_val, db_item, item_size) == 0) {
            /* Key matches with the one in RAM database */
            return i;
        }
        db_item += item_size;
    }
    return -1;
}

static int
get_nvs_max_obj_value(int obj_type)
{
    /* If host based privacy is enabled */
    if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
        return (MYNEWT_VAL(BLE_STORE_MAX_BONDS) + 1);
    } else {
        if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
            return MYNEWT_VAL(BLE_STORE_MAX_CCCDS);
        } else {
            return MYNEWT_VAL(BLE_STORE_MAX_BONDS);
        }
    }
}

/*****************************************************************************
 * $ NVS                                                                     *
 *****************************************************************************/
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
static int
get_nvs_peer_record(char *key_string, struct ble_hs_dev_records *p_dev_rec)
{
    esp_err_t err;
    size_t required_size = 0;
    nvs_handle_t nimble_handle;

    err = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return BLE_HS_ESTORE_FAIL;
    }

    err = nvs_get_blob(nimble_handle, key_string, NULL, &required_size);

    /* if Address pointer for value is NULL, filling of value not needed */
    if (err != ESP_OK || p_dev_rec == NULL) {
        goto end;
    }

    err = nvs_get_blob(nimble_handle, key_string, p_dev_rec,
                       &required_size);

end:
    nvs_close(nimble_handle);
    return err;
}
#endif

static int
get_nvs_db_value(int obj_type, char *key_string, union ble_store_value *val)
{
    esp_err_t err;
    size_t required_size = 0;
    nvs_handle_t nimble_handle;

    err = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return BLE_HS_ESTORE_FAIL;
    }

    err = nvs_get_blob(nimble_handle, key_string, NULL, &required_size);

    /* if Address pointer for value is NULL, filling of value not needed */
    if (err != ESP_OK || val == NULL) {
        goto end;
    }

    if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
        err = nvs_get_blob(nimble_handle, key_string, &val->cccd,
                           &required_size);
    } else {
        err = nvs_get_blob(nimble_handle, key_string, &val->sec,
                           &required_size);
    }

end:
    nvs_close(nimble_handle);
    return err;
}

/* Finds empty index or total count or index to be deleted in NVS database
* This function serves 3 different purposes depending upon 'empty' and `value`
* arguments.
* @ returns             - empty NVS index, if empty = 1
*                       - count of NVS database, if empty = 0, value = NULL
*                       - index that does not match with RAM db, if empty = 0 &
*                         value has valid database address.
*/
static int
get_nvs_db_attribute(int obj_type, bool empty, void *value, int num_value)
{
    union ble_store_value cur = {0};
    struct ble_hs_dev_records p_dev_rec = {0};
    esp_err_t err;
    int i, count = 0, max_limit = 0;
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

    max_limit = get_nvs_max_obj_value(obj_type);

    for (i = 1; i <= max_limit; i++) {
        get_nvs_key_string(obj_type, i, key_string);

        if (obj_type != BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
            err = get_nvs_db_value(obj_type, key_string, &cur);
        } else {
            err = get_nvs_peer_record(key_string, &p_dev_rec);
        }
        /* Check if the user is searching for empty index to write to */
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            if (empty) {
                ESP_LOGD(TAG, "Empty NVS index found = %d for obj_type = %d", i, obj_type);
                return i;
            }
        } else if (err == ESP_OK) {
            count++;
            /* If user has provided value, then the purpose is to find
             * non-matching entry from NVS */
            if (value) {
                if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
                    err = get_nvs_matching_index(&p_dev_rec, value, num_value,
                                                 sizeof(struct ble_hs_dev_records));
                } else {
                    if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
                        err = get_nvs_matching_index(&cur.sec, value, num_value,
                                                     sizeof(struct ble_store_value_sec));
                    } else {
                        err = get_nvs_matching_index(&cur.sec, value, num_value,
                                                     sizeof(struct ble_store_value_cccd));
                    }
                }
                /* If found non-matching/odd entry of NVS with entries in the
                 * internal database, return NVS index so can be deleted */
                if (err == -1 && !empty) {
                    return i;
                }
            }
        } else {
            ESP_LOGE(TAG, "NVS read operation failed while fetching size !!");
            return -1;
        }
    }

    if (empty == 0) {
        return count;
    } else {
        return (max_limit + 1);
    }
}

/* Deletes NVS value at given index
* @Returns               0 on success,
*                       -1 on NVS memory access failure
*/
static int
ble_nvs_delete_value(int obj_type, int8_t index)
{
    esp_err_t err;
    nvs_handle_t nimble_handle;
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

    if (index > get_nvs_max_obj_value(obj_type)) {
        ESP_LOGE(TAG, "Invalid index provided to delete");
        return BLE_HS_EUNKNOWN;
    }

    err = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed !!");
        return BLE_HS_ESTORE_FAIL;
    }

    get_nvs_key_string(obj_type, index, key_string);

    /* Erase the key with given index */
    err = nvs_erase_key(nimble_handle, key_string);
    if (err != ESP_OK) {
        goto error;
    }
    err = nvs_commit(nimble_handle);
    if (err != ESP_OK) {
        goto error;
    }

    nvs_close(nimble_handle);
    return 0;
error:
    nvs_close(nimble_handle);
    return BLE_HS_ESTORE_FAIL;
}

static int
ble_nvs_write_key_value(char *key, const void *value, size_t required_size)
{
    nvs_handle_t nimble_handle;
    esp_err_t err;

    err = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed !!");
        return BLE_HS_ESTORE_FAIL;
    }

    err = nvs_set_blob(nimble_handle, key, value, required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS write operation failed !!");
        goto error;
    }

    /* NVS commit and close */
    err = nvs_commit(nimble_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit operation failed !!");
        goto error;
    }

    nvs_close(nimble_handle);
    return 0;
error:
    nvs_close(nimble_handle);
    return BLE_HS_ESTORE_FAIL;
}

/* To write key value in NVS.
* @Returns              0 if success
*                       BLE_HS_ESTORE_FAIL if failure
*                       BLE_HS_ESTORE_CAP if no space in NVS
*/
static int
ble_store_nvs_write(int obj_type, const union ble_store_value *val)
{
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
    int8_t write_key_index = 0;

    write_key_index = get_nvs_db_attribute(obj_type, 1, NULL, 0);
    if (write_key_index == -1) {
        ESP_LOGE(TAG, "NVS operation failed !!");
        return BLE_HS_ESTORE_FAIL;
    } else if (write_key_index > get_nvs_max_obj_value(obj_type)) {

        /* bare-bone config code will take care of capacity overflow event,
         * however another check added for consistency */
        ESP_LOGD(TAG, "NVS size overflow.");
        return BLE_HS_ESTORE_CAP;
    }

    get_nvs_key_string(obj_type, write_key_index, key_string);

    if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
        return ble_nvs_write_key_value(key_string, &val->cccd, sizeof(struct
                                       ble_store_value_cccd));
    } else {
        return ble_nvs_write_key_value(key_string, &val->sec, sizeof(struct
                                       ble_store_value_sec));
    }
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
/* If Host based privacy is enabled */
static int
ble_store_nvs_peer_records(int obj_type, const struct ble_hs_dev_records *p_dev_rec)
{
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
    int8_t write_key_index = 0;

    write_key_index = get_nvs_db_attribute(obj_type, 1, NULL, 0);
    if (write_key_index == -1) {
        ESP_LOGE(TAG, "NVS operation failed !!");
        return BLE_HS_ESTORE_FAIL;
    } else if (write_key_index > get_nvs_max_obj_value(obj_type)) {

        /* bare-bone config code will take care of capacity overflow event,
         * however another check added for consistency */
        ESP_LOGD(TAG, "NVS size overflow.");
        return BLE_HS_ESTORE_CAP;
    }

    get_nvs_key_string(obj_type, write_key_index, key_string);

    return ble_nvs_write_key_value(key_string, p_dev_rec, sizeof(struct
                                   ble_hs_dev_records));
}
#endif

static int
populate_db_from_nvs(int obj_type, void *dst, int *db_num)
{
    uint8_t *db_item = (uint8_t *)dst;
    union ble_store_value cur = {0};
    struct ble_hs_dev_records p_dev_rec = {0};

    esp_err_t err;
    int i;
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

    for (i = 1; i <= get_nvs_max_obj_value(obj_type); i++) {
        get_nvs_key_string(obj_type, i, key_string);

        if (obj_type != BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {

            err = get_nvs_db_value(obj_type, key_string, &cur);
            if (err == ESP_ERR_NVS_NOT_FOUND) {
                continue;
            } else if (err != ESP_OK) {
                ESP_LOGE(TAG, "NVS read operation failed !!");
                return -1;
            }
        } else {
            err = get_nvs_peer_record(key_string, &p_dev_rec);
            if (err == ESP_ERR_NVS_NOT_FOUND) {
                continue;
            } else if (err != ESP_OK) {
                ESP_LOGE(TAG, "NVS read operation failed !!");
                return -1;
            }
        }

        /* NVS index has data, fill up the ram db with it */
        if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
            ESP_LOGD(TAG, "Peer dev records filled from NVS index = %d", i);
            memcpy(db_item, &p_dev_rec, sizeof(struct ble_hs_dev_records));
            db_item += sizeof(struct ble_hs_dev_records);
            (*db_num)++;
        } else {
            if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
                ESP_LOGD(TAG, "CCCD in RAM is filled up from NVS index = %d", i);
                memcpy(db_item, &cur.cccd, sizeof(struct ble_store_value_cccd));
                db_item += sizeof(struct ble_store_value_cccd);
                (*db_num)++;
            } else {
                ESP_LOGD(TAG, "KEY in RAM is filled up from NVS index = %d", i);
                memcpy(db_item, &cur.sec, sizeof(struct ble_store_value_sec));
                db_item += sizeof(struct ble_store_value_sec);
                (*db_num)++;
            }
        }
    }
    return 0;
}

/* Gets the database in RAM filled up with keys stored in NVS. The sequence of
 * the keys in database may get lost.
 */
static int
ble_nvs_restore_sec_keys(void)
{
    esp_err_t err;

    err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_OUR_SEC, ble_store_config_our_secs,
                               &ble_store_config_num_our_secs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed for 'our sec'");
        return err;
    }
    ESP_LOGD(TAG, "ble_store_config_our_secs restored %d bonds", ble_store_config_num_our_secs);

    err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_PEER_SEC, ble_store_config_peer_secs,
                               &ble_store_config_num_peer_secs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed for 'peer sec'");
        return err;
    }
    ESP_LOGD(TAG, "ble_store_config_peer_secs restored %d bonds",
             ble_store_config_num_peer_secs);

    err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_CCCD, ble_store_config_cccds,
                               &ble_store_config_num_cccds);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed for 'CCCD'");
        return err;
    }
    ESP_LOGD(TAG, "ble_store_config_cccds restored %d bonds",
             ble_store_config_num_cccds);

    return 0;
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
static int
ble_nvs_restore_peer_records(void)
{
    esp_err_t err;
    int ble_store_num_peer_dev_rec = 0;
    struct ble_hs_dev_records *peer_dev_rec = ble_rpa_get_peer_dev_records();

    err = populate_db_from_nvs(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, peer_dev_rec,
                               &ble_store_num_peer_dev_rec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed fetching 'Peer Dev Records'");
        return err;
    }

    ble_rpa_set_num_peer_dev_records(ble_store_num_peer_dev_rec);
    ESP_LOGD(TAG, "peer_dev_rec restored %d records", ble_store_num_peer_dev_rec);

    return 0;
}
#endif

int ble_store_config_persist_cccds(void)
{
    int nvs_count, nvs_idx;
    union ble_store_value val;

    nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_CCCD, 0, NULL, 0);
    if (nvs_count < ble_store_config_num_cccds) {

        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting CCCD value in NVS...");
        val.cccd = ble_store_config_cccds[ble_store_config_num_cccds - 1];
        return ble_store_nvs_write(BLE_STORE_OBJ_TYPE_CCCD, &val);
    } else if (nvs_count > ble_store_config_num_cccds) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_CCCD, 0,
                                       ble_store_config_cccds, ble_store_config_num_cccds);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for CCCD");
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting CCCD, nvs idx = %d", nvs_idx);
        return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_CCCD, nvs_idx);
    }
    return 0;
}

int ble_store_config_persist_peer_secs(void)
{
    int nvs_count, nvs_idx;
    union ble_store_value val;

    nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_PEER_SEC, 0, NULL, 0);
    if (nvs_count < ble_store_config_num_peer_secs) {

        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting peer sec value in NVS...");
        val.sec = ble_store_config_peer_secs[ble_store_config_num_peer_secs - 1];
        return ble_store_nvs_write(BLE_STORE_OBJ_TYPE_PEER_SEC, &val);
    } else if (nvs_count > ble_store_config_num_peer_secs) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_PEER_SEC, 0,
                                       ble_store_config_peer_secs, ble_store_config_num_peer_secs);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for peer sec");
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting peer sec, nvs idx = %d", nvs_idx);
        return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_PEER_SEC, nvs_idx);
    }
    return 0;
}

int ble_store_config_persist_our_secs(void)
{
    int nvs_count, nvs_idx;
    union ble_store_value val;

    nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_OUR_SEC, 0, NULL, 0);
    if (nvs_count < ble_store_config_num_our_secs) {

        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting our sec value to NVS...");
        val.sec = ble_store_config_our_secs[ble_store_config_num_our_secs - 1];
        return ble_store_nvs_write(BLE_STORE_OBJ_TYPE_OUR_SEC, &val);
    } else if (nvs_count > ble_store_config_num_our_secs) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_OUR_SEC, 0,
                                       ble_store_config_our_secs, ble_store_config_num_our_secs);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for our sec");
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting our sec, nvs idx = %d", nvs_idx);
        return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_OUR_SEC, nvs_idx);
    }
    return 0;
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
int ble_store_persist_peer_records(void)
{
    int nvs_count, nvs_idx;
    struct ble_hs_dev_records peer_rec;
    int ble_store_num_peer_dev_rec = ble_rpa_get_num_peer_dev_records();
    struct ble_hs_dev_records *peer_dev_rec = ble_rpa_get_peer_dev_records();

    nvs_count = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, 0, NULL, 0);
    if (nvs_count < ble_store_num_peer_dev_rec) {
        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting peer dev record to NVS...");
        peer_rec = peer_dev_rec[ble_store_num_peer_dev_rec - 1];
        return ble_store_nvs_peer_records(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, &peer_rec);
    } else if (nvs_count > ble_store_num_peer_dev_rec) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, 0,
                                       peer_dev_rec,
                                       ble_store_num_peer_dev_rec);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for peer records");
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting peer record, nvs idx = %d", nvs_idx);
        return ble_nvs_delete_value(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, nvs_idx);
    }
    return 0;
}
#endif

void ble_store_config_conf_init(void)
{
    int err;

    err = ble_nvs_restore_sec_keys();
    if (err != 0) {
        ESP_LOGE(TAG, "NVS operation failed, can't retrieve the bonding info");
    }

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
    err = ble_nvs_restore_peer_records();
    if (err != 0) {
        ESP_LOGE(TAG, "NVS operation failed, can't retrieve the peer records");
    }
#endif

    bluedroid_nvs_to_nimble_nvs();

}

#include <ctype.h>

/// Used to exchange the encryption key in the init key & response key
#define ESP_BLE_ENC_KEY_MASK    (1 << 0)            /* relate to BTM_BLE_ENC_KEY_MASK in stack/btm_api.h */
/// Used to exchange the IRK key in the init key & response key
#define ESP_BLE_ID_KEY_MASK     (1 << 1)            /* relate to BTM_BLE_ID_KEY_MASK in stack/btm_api.h */
/// Used to exchange the CSRK key in the init key & response key
#define ESP_BLE_CSR_KEY_MASK    (1 << 2)            /* relate to BTM_BLE_CSR_KEY_MASK in stack/btm_api.h */
/// Used to exchange the link key(this key just used in the BLE & BR/EDR coexist mode) in the init key & response key
#define ESP_BLE_LINK_KEY_MASK   (1 << 3)            /* relate to BTM_BLE_LINK_KEY_MASK in stack/btm_api.h */
typedef uint8_t esp_ble_key_mask_t;            /* the key mask type */

/*Define the bt octet 16 bit size*/
#define ESP_BT_OCTET16_LEN    16
typedef uint8_t esp_bt_octet16_t[ESP_BT_OCTET16_LEN];   /* octet array: size 16 */

#define ESP_BT_OCTET8_LEN    8
typedef uint8_t esp_bt_octet8_t[ESP_BT_OCTET8_LEN];   /* octet array: size 8 */

/*Bluetooth Address*/
typedef struct {
    uint8_t address[6];
} __attribute__ ((__packed__)) bt_bdaddr_t;

/// Bluetooth address length
#define ESP_BD_ADDR_LEN     6

/// Bluetooth device address
typedef uint8_t esp_bd_addr_t[ESP_BD_ADDR_LEN];

/// BLE device address type
typedef enum {
    BLE_ADDR_TYPE_PUBLIC        = 0x00,
    BLE_ADDR_TYPE_RANDOM        = 0x01,
    BLE_ADDR_TYPE_RPA_PUBLIC    = 0x02,
    BLE_ADDR_TYPE_RPA_RANDOM    = 0x03,
} esp_ble_addr_type_t;

/**
* @brief  BLE Encryption reproduction keys
*/
typedef struct
{
    esp_bt_octet16_t  ltk;                  /*!< The long term key */
    uint16_t          div;                  /*!< The div value */
    uint8_t           key_size;             /*!< The key size of the security link */
    uint8_t           sec_level;            /*!< The security level of the security link */
} esp_ble_lenc_keys_t;                      /*!< The  key type */

/**
* @brief  BLE SRK keys
*/
typedef struct
{
    uint32_t          counter;              /*!< The counter value */
    uint16_t          div;                  /*!< The div value */
    uint8_t           sec_level;            /*!< The security level of the security link */
    esp_bt_octet16_t  csrk;                 /*!< The csrk key value */
} esp_ble_lcsrk_keys;                       /*!< The csrk key type */

/**
* @brief BLE encryption keys
*/
typedef struct
{
    esp_bt_octet16_t     ltk;          /*!< The long term key*/
    esp_bt_octet8_t      rand;         /*!< The random number*/
    uint16_t             ediv;         /*!< The ediv value*/
    uint8_t              sec_level;    /*!< The security level of the security link*/
    uint8_t              key_size;     /*!< The key size(7~16) of the security link*/
} esp_ble_penc_keys_t;                 /*!< The key type*/

/**
* @brief  BLE CSRK keys
*/
typedef struct
{
    uint32_t            counter;      /*!< The counter */
    esp_bt_octet16_t    csrk;         /*!< The csrk key */
    uint8_t             sec_level;    /*!< The security level */
} esp_ble_pcsrk_keys_t;               /*!< The pcsrk key type */

/**
* @brief  BLE pid keys
*/
typedef struct
{
    esp_bt_octet16_t          irk;           /*!< The irk value */
    esp_ble_addr_type_t       addr_type;     /*!< The address type */
    esp_bd_addr_t             static_addr;   /*!< The static address */
} esp_ble_pid_keys_t;                        /*!< The pid key type */

/**
* @brief  struct type of the bond key information value
*/
typedef struct
{
    esp_ble_key_mask_t    key_mask;       /*!< the key mask to indicate witch key is present */
    esp_ble_penc_keys_t   penc_key;       /*!< received peer encryption key */
    esp_ble_pcsrk_keys_t  pcsrk_key;      /*!< received peer device SRK */
    esp_ble_pid_keys_t    pid_key;        /*!< peer device ID key */
    esp_ble_lcsrk_keys    lcsrk_key;
    esp_ble_lenc_keys_t   lenck_key;
    esp_ble_pid_keys_t    lid_key;

} esp_ble_bond_key_info_t;                /*!< ble bond key information value type */

/**
* @brief  struct type of the bond device value
*/
typedef struct
{
    esp_bd_addr_t  bd_addr;               /*!< peer address */
    esp_ble_bond_key_info_t bond_key;     /*!< the bond key information */
} esp_ble_bond_dev_t;                     /*!< the ble bond device type */

typedef uint8_t UINT8;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
typedef uint64_t UINT64;

typedef int8_t INT8;
typedef int16_t INT16;
typedef int32_t INT32;
typedef bool BOOLEAN;

#define BT_OCTET8_LEN    8
typedef UINT8 BT_OCTET8[BT_OCTET8_LEN];   /* octet array: size 16 */

#define BT_OCTET16_LEN    16
typedef UINT8 BT_OCTET16[BT_OCTET16_LEN];   /* octet array: size 16 */

/* Common Bluetooth field definitions */
#define BD_ADDR_LEN     6                   /* Device address length */
typedef UINT8 BD_ADDR[BD_ADDR_LEN];         /* Device address */
typedef UINT8 *BD_ADDR_PTR;                 /* Pointer to Device Address */

//#define BLE_ADDR_PUBLIC         0x00
//#define BLE_ADDR_RANDOM         0x01
//#define BLE_ADDR_PUBLIC_ID      0x02
//#define BLE_ADDR_RANDOM_ID      0x03
//#define BLE_ADDR_TYPE_MAX       BLE_ADDR_RANDOM_ID
//#define BLE_ADDR_UNKNOWN_TYPE   0XFF
typedef UINT8 tBLE_ADDR_TYPE;
#define BLE_ADDR_TYPE_MASK      (BLE_ADDR_RANDOM | BLE_ADDR_PUBLIC)

/* BLE encryption keys */
typedef struct {
    BT_OCTET16  ltk;
    BT_OCTET8   rand;
    UINT16      ediv;
    UINT8       sec_level;
    UINT8       key_size;
} tBTM_LE_PENC_KEYS;

/* BLE CSRK keys */
typedef struct {
    UINT32          counter;
    BT_OCTET16      csrk;
    UINT8           sec_level;
} tBTM_LE_PCSRK_KEYS;

/* BLE Encryption reproduction keys */
typedef struct {
    BT_OCTET16  ltk;
    UINT16      div;
    UINT8       key_size;
    UINT8       sec_level;
} tBTM_LE_LENC_KEYS;

/* BLE SRK keys */
typedef struct {
    UINT32          counter;
    UINT16          div;
    UINT8           sec_level;
    BT_OCTET16      csrk;
} tBTM_LE_LCSRK_KEYS;

typedef struct {
    BT_OCTET16          irk;
    tBLE_ADDR_TYPE      addr_type;
    BD_ADDR             static_addr;
} tBTM_LE_PID_KEYS;

typedef union {
    tBTM_LE_PENC_KEYS   penc_key;       /* received peer encryption key */
    tBTM_LE_PCSRK_KEYS  pcsrk_key;      /* received peer device SRK */
    tBTM_LE_PID_KEYS    pid_key;        /* peer device ID key */
    tBTM_LE_LENC_KEYS   lenc_key;       /* local encryption reproduction keys LTK = = d1(ER,DIV,0)*/
    tBTM_LE_LCSRK_KEYS   lcsrk_key;     /* local device CSRK = d1(ER,DIV,1)*/
} tBTM_LE_KEY_VALUE;

/** Bluetooth Error Status */
/** We need to build on this */

/* relate to ESP_BT_STATUS_xxx in esp_bt_defs.h */
typedef enum {
    BT_STATUS_SUCCESS = 0,
    BT_STATUS_FAIL,
    BT_STATUS_NOT_READY,
    BT_STATUS_NOMEM,
    BT_STATUS_BUSY,
    BT_STATUS_DONE,        /* request already completed */
    BT_STATUS_UNSUPPORTED,
    BT_STATUS_PARM_INVALID,
    BT_STATUS_UNHANDLED,
    BT_STATUS_AUTH_FAILURE,
    BT_STATUS_RMT_DEV_DOWN,
    BT_STATUS_AUTH_REJECTED,
    BT_STATUS_INVALID_STATIC_RAND_ADDR,
    BT_STATUS_PENDING,
    BT_STATUS_UNACCEPT_CONN_INTERVAL,
    BT_STATUS_PARAM_OUT_OF_RANGE,
    BT_STATUS_TIMEOUT,
    BT_STATUS_MEMORY_FULL,
    BT_STATUS_EIR_TOO_LARGE,
} bt_status_t;

/* SMP key types */
#define SMP_SEC_KEY_TYPE_ENC                (1 << 0)    /* encryption key */
#define SMP_SEC_KEY_TYPE_ID                 (1 << 1)    /* identity key */
#define SMP_SEC_KEY_TYPE_CSRK               (1 << 2)    /* slave CSRK */
#define SMP_SEC_KEY_TYPE_LK                 (1 << 3)    /* BR/EDR link key */
typedef UINT8 tSMP_KEYS;

#define BTM_LE_KEY_NONE           0
#define BTM_LE_KEY_PENC      SMP_SEC_KEY_TYPE_ENC        /* encryption information of peer device */
#define BTM_LE_KEY_PID       SMP_SEC_KEY_TYPE_ID         /* identity key of the peer device */
#define BTM_LE_KEY_PCSRK     SMP_SEC_KEY_TYPE_CSRK      /* peer SRK */
#define BTM_LE_KEY_PLK       SMP_SEC_KEY_TYPE_LK
#define BTM_LE_KEY_LLK       (SMP_SEC_KEY_TYPE_LK << 4)
#define BTM_LE_KEY_LENC      (SMP_SEC_KEY_TYPE_ENC << 4)  /* master role security information:div */
#define BTM_LE_KEY_LID       (SMP_SEC_KEY_TYPE_ID << 4)   /* master device ID key */
#define BTM_LE_KEY_LCSRK     (SMP_SEC_KEY_TYPE_CSRK << 4) /* local CSRK has been deliver to peer */

#define BT_DEVICE_TYPE_BREDR   0x01
#define BT_DEVICE_TYPE_BLE     0x02
#define BT_DEVICE_TYPE_DUMO    0x03

#define BTC_BLE_STORAGE_DEV_TYPE_STR                "DevType"

#define osi_malloc(size)                  malloc((size))
#define osi_calloc(size)                  calloc(1, (size))
#define osi_free(p)                       free((p))

typedef void (*list_free_cb)(void *data);

typedef struct  list_node_t list_node_t;

struct list_node_t {
    struct list_node_t *next;
    void *data;
};

typedef struct list_t {
    list_node_t *head;
    list_node_t *tail;
    size_t length;
    list_free_cb free_cb;
} list_t;

typedef struct {
    char *key;
    char *value;
} entry_t;

typedef struct {
    char *name;
    list_t *entries;
} section_t;

struct config_t {
    list_t *sections;
};

char *osi_strdup(const char *str)
{
    size_t size = strlen(str) + 1;  // + 1 for the null terminator
    char *new_string = (char *)osi_calloc(size);

    if (!new_string) {
        return NULL;
    }

    memcpy(new_string, str, size);
    return new_string;
}

static char *trim(char *str)
{
    while (isspace((unsigned char)(*str))) {
        ++str;
    }

    if (!*str) {
        return str;
    }

    char *end_str = str + strlen(str) - 1;
    while (end_str > str && isspace((unsigned char)(*end_str))) {
        --end_str;
    }

    end_str[1] = '\0';
    return str;
}

// == from btc_config.h
typedef struct btc_config_section_iter_t btc_config_section_iter_t;
typedef struct config_section_node_t config_section_node_t;
typedef struct config_t config_t;

static config_t *config;

list_t *list_new_internal(list_free_cb callback)
{
    list_t *list = (list_t *) osi_calloc(sizeof(list_t));
    if (!list) {
        return NULL;
    }

    list->head = list->tail = NULL;
    list->length = 0;
    list->free_cb = callback;
    return list;
}

list_t *list_new(list_free_cb callback)
{
    return list_new_internal(callback);
}

list_node_t *list_free_node(list_t *list, list_node_t *node)
{
    assert(list != NULL);
    assert(node != NULL);

    list_node_t *next = node->next;

    if (list->free_cb) {
        list->free_cb(node->data);
    }
    free(node);
    --list->length;

    return next;
}


void list_clear(list_t *list)
{
    assert(list != NULL);
    for (list_node_t *node = list->head; node; ) {
        node = list_free_node(list, node);
    }
    list->head = NULL;
    list->tail = NULL;
    list->length = 0;
}

void list_free(list_t *list)
{
    if (!list) {
        return;
    }

    list_clear(list);
    osi_free(list);
}

list_node_t *list_begin(const list_t *list)
{
    assert(list != NULL);
    return list->head;
}

list_node_t *list_end(const list_t *list)
{
    assert(list != NULL);
    return NULL;
}

list_node_t *list_next(const list_node_t *node)
{
    assert(node != NULL);
    return node->next;
}

void *list_node(const list_node_t *node)
{
    assert(node != NULL);
    return node->data;
}

bool list_prepend(list_t *list, void *data)
{
    assert(list != NULL);
    assert(data != NULL);
    list_node_t *node = (list_node_t *)osi_calloc(sizeof(list_node_t));
    if (!node) {
        ESP_LOGE("OSI", "%s osi_calloc failed.\n", __FUNCTION__ );
        return false;
    }
    node->next = list->head;
    node->data = data;
    list->head = node;
    if (list->tail == NULL) {
        list->tail = list->head;
    }
    ++list->length;
    return true;
}

bool list_append(list_t *list, void *data)
{
    assert(list != NULL);
    assert(data != NULL);
    list_node_t *node = (list_node_t *)osi_calloc(sizeof(list_node_t));
    if (!node) {
        ESP_LOGE("OSI", "%s osi_calloc failed.\n", __FUNCTION__ );
        return false;
    }
    node->next = NULL;
    node->data = data;
    if (list->tail == NULL) {
        list->head = node;
        list->tail = node;
    } else {
        list->tail->next = node;
        list->tail = node;
    }
    ++list->length;
    return true;
}

// == from config.c
void config_free(config_t *config)
{
    if (!config) {
        return;
    }

    list_free(config->sections);
    osi_free(config);
}

const config_section_node_t *config_section_begin(const config_t *config)
{
    assert(config != NULL);
    return (const config_section_node_t *)list_begin(config->sections);
}

const config_section_node_t *config_section_end(const config_t *config)
{
    assert(config != NULL);
    return (const config_section_node_t *)list_end(config->sections);
}

const config_section_node_t *config_section_next(const config_section_node_t *node)
{
    assert(node != NULL);
    return (const config_section_node_t *)list_next((const list_node_t *)node);
}

const char *config_section_name(const config_section_node_t *node)
{
    assert(node != NULL);
    const list_node_t *lnode = (const list_node_t *)node;
    const section_t *section = (const section_t *)list_node(lnode);
    return section->name;
}

static void entry_free(void *ptr)
{
    if (!ptr) {
        return;
    }

    entry_t *entry = ptr;
    osi_free(entry->key);
    osi_free(entry->value);
    osi_free(entry);
}

static section_t *section_new(const char *name)
{
    section_t *section = osi_calloc(sizeof(section_t));
    if (!section) {
        return NULL;
    }

    section->name = osi_strdup(name);
    section->entries = list_new(entry_free);
    return section;
}

static void section_free(void *ptr)
{
    if (!ptr) {
        return;
    }

    section_t *section = ptr;
    osi_free(section->name);
    list_free(section->entries);
    osi_free(section);
}

static section_t *section_find(const config_t *config, const char *section)
{
    for (const list_node_t *node = list_begin(config->sections); node != list_end(config->sections); node = list_next(node)) {
        section_t *sec = list_node(node);
        if (!strcmp(sec->name, section)) {
            return sec;
        }
    }

    return NULL;
}

static entry_t *entry_new(const char *key, const char *value)
{
    entry_t *entry = osi_calloc(sizeof(entry_t));
    if (!entry) {
        return NULL;
    }

    entry->key = osi_strdup(key);
    entry->value = osi_strdup(value);
    return entry;
}



static entry_t *entry_find(const config_t *config, const char *section, const char *key)
{
    section_t *sec = section_find(config, section);
    if (!sec) {
        return NULL;
    }

    for (const list_node_t *node = list_begin(sec->entries); node != list_end(sec->entries); node = list_next(node)) {
        entry_t *entry = list_node(node);
        if (!strcmp(entry->key, key)) {
            return entry;
        }
    }

    return NULL;
}

bool config_has_key(const config_t *config, const char *section, const char *key)
{
    assert(config != NULL);
    assert(section != NULL);
    assert(key != NULL);

    return (entry_find(config, section, key) != NULL);
}

void config_set_string(config_t *config, const char *section, const char *key, const char *value, bool insert_back)
{
    section_t *sec = section_find(config, section);
    if (!sec) {
        sec = section_new(section);
        if (insert_back) {
            list_append(config->sections, sec);
        } else {
            list_prepend(config->sections, sec);
        }
    }

    for (const list_node_t *node = list_begin(sec->entries); node != list_end(sec->entries); node = list_next(node)) {
        entry_t *entry = list_node(node);
        if (!strcmp(entry->key, key)) {
            osi_free(entry->value);
            entry->value = osi_strdup(value);
            return;
        }
    }

    entry_t *entry = entry_new(key, value);
    list_append(sec->entries, entry);
}

const char *config_get_string(const config_t *config, const char *section, const char *key, const char *def_value)
{
    assert(config != NULL);
    assert(section != NULL);
    assert(key != NULL);

    entry_t *entry = entry_find(config, section, key);
    if (!entry) {
        return def_value;
    }

    return entry->value;
}

int config_get_int(const config_t *config, const char *section, const char *key, int def_value)
{
    assert(config != NULL);
    assert(section != NULL);
    assert(key != NULL);

    entry_t *entry = entry_find(config, section, key);
    if (!entry) {
        return def_value;
    }

    char *endptr;
    int ret = strtol(entry->value, &endptr, 0);
    return (*endptr == '\0') ? ret : def_value;
}


config_t *config_new_empty(void)
{
    config_t *config = osi_calloc(sizeof(config_t));
    if (!config) {
        ESP_LOGE("OSI", "%s unable to allocate memory for config_t.\n", __func__);
        goto error;
    }

    config->sections = list_new(section_free);
    if (!config->sections) {
        ESP_LOGE("OSI", "%s unable to allocate list for sections.\n", __func__);
        goto error;
    }

    return config;

error:;
    config_free(config);
    return NULL;
}

// BTC config stuff


static inline bool ets_isxdigit(char c)
{
    if ((c >= '0') && (c <= '9')) {
        return true;
    }
    if ((c >= 'a') && (c <= 'f')) {
        return true;
    }
    return ((c >= 'A') && (c <= 'F'));
}

bool btc_config_get_bin(const char *section, const char *key, uint8_t *value, size_t *length)
{
    assert(config != NULL);
    assert(section != NULL);
    assert(key != NULL);
    assert(value != NULL);
    assert(length != NULL);

    const char *value_str = config_get_string(config, section, key, NULL);

    if (!value_str) {
        return false;
    }

    size_t value_len = strlen(value_str);
    if ((value_len % 2) != 0 || *length < (value_len / 2)) {
        return false;
    }

    for (size_t i = 0; i < value_len; ++i)
        if (!isxdigit((unsigned char)value_str[i])) {
            return false;
        }

    for (*length = 0; *value_str; value_str += 2, *length += 1) {
        unsigned int val;
        sscanf(value_str, "%02x", &val);
        value[*length] = (uint8_t)(val);
    }

    return true;
}

const btc_config_section_iter_t *btc_config_section_begin(void)
{
    assert(config != NULL);
    return (const btc_config_section_iter_t *)config_section_begin(config);
}

const btc_config_section_iter_t *btc_config_section_end(void)
{
    assert(config != NULL);
    return (const btc_config_section_iter_t *)config_section_end(config);
}

const btc_config_section_iter_t *btc_config_section_next(const btc_config_section_iter_t *section)
{
    assert(config != NULL);
    assert(section != NULL);
    return (const btc_config_section_iter_t *)config_section_next((const config_section_node_t *)section);
}

const char *btc_config_section_name(const btc_config_section_iter_t *section)
{
    assert(config != NULL);
    assert(section != NULL);
    return config_section_name((const config_section_node_t *)section);
}

void btc_config_lock(void)
{
    //osi_mutex_lock(&lock, OSI_MUTEX_MAX_TIMEOUT);
}

void btc_config_unlock(void)
{
    //osi_mutex_unlock(&lock);
}

bool btc_config_get_int(const char *section, const char *key, int *value)
{
    assert(config != NULL);
    assert(section != NULL);
    assert(key != NULL);
    assert(value != NULL);

    bool ret = config_has_key(config, section, key);
    if (ret) {
        *value = config_get_int(config, section, key, *value);
    }

    return ret;
}


bool string_is_bdaddr(const char *string)
{
    assert(string != NULL);

    size_t len = strlen(string);
    if (len != 17) {
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        // Every 3rd char must be ':'.
        if (((i + 1) % 3) == 0 && string[i] != ':') {
            return false;
        }

        // All other chars must be a hex digit.
        if (((i + 1) % 3) != 0 && !ets_isxdigit(string[i])) {
            return false;
        }
    }
    return true;
}

bool string_to_bdaddr(const char *string, bt_bdaddr_t *addr)
{
    assert(string != NULL);
    assert(addr != NULL);

    bt_bdaddr_t new_addr;
    uint8_t *ptr = new_addr.address;
    uint32_t ptr_32[6];
    bool ret  = sscanf(string, "%02x:%02x:%02x:%02x:%02x:%02x",
                      &ptr_32[0], &ptr_32[1], &ptr_32[2], &ptr_32[3], &ptr_32[4], &ptr_32[5]) == 6;
    if (ret) {
        for (uint8_t i = 0; i < 6; i++){
            ptr[i] = (uint8_t) ptr_32[i];
        }
        memcpy(addr, &new_addr, sizeof(bt_bdaddr_t));
    }

    return ret;
}

const char *bdaddr_to_string(const bt_bdaddr_t *addr, char *string, size_t size)
{
    assert(addr != NULL);
    assert(string != NULL);

    if (size < 18) {
        return NULL;
    }

    const uint8_t *ptr = addr->address;
    sprintf(string, "%02x:%02x:%02x:%02x:%02x:%02x",
            ptr[0], ptr[1], ptr[2],
            ptr[3], ptr[4], ptr[5]);
    return string;
}

typedef char bdstr_t[18];

#define BTC_BLE_STORAGE_DEV_TYPE_STR                "DevType"
#define BTC_BLE_STORAGE_ADDR_TYPE_STR               "AddrType"
#define BTC_BLE_STORAGE_LINK_KEY_STR                "LinkKey"
#define BTC_BLE_STORAGE_LE_KEY_PENC_STR             "LE_KEY_PENC"
#define BTC_BLE_STORAGE_LE_KEY_PID_STR              "LE_KEY_PID"
#define BTC_BLE_STORAGE_LE_KEY_PCSRK_STR            "LE_KEY_PCSRK"
#define BTC_BLE_STORAGE_LE_KEY_LENC_STR             "LE_KEY_LENC"
#define BTC_BLE_STORAGE_LE_KEY_LID_STR              "LE_KEY_LID"
#define BTC_BLE_STORAGE_LE_KEY_LCSRK_STR            "LE_KEY_LCSRK"
#define BTC_BLE_STORAGE_LE_AUTH_MODE_STR            "AuthMode"

static bt_status_t _btc_storage_get_ble_bonding_key(bt_bdaddr_t *remote_bd_addr,
                                            uint8_t key_type,
                                            char *key_value,
                                            int key_length)
{
    bdstr_t bdstr;
    bdaddr_to_string(remote_bd_addr, bdstr, sizeof(bdstr));
    const char* name;
    switch (key_type) {
    case BTM_LE_KEY_PENC:
        name = BTC_BLE_STORAGE_LE_KEY_PENC_STR;
        break;
    case BTM_LE_KEY_PID:
        name = BTC_BLE_STORAGE_LE_KEY_PID_STR;
        break;
    case BTM_LE_KEY_PCSRK:
        name = BTC_BLE_STORAGE_LE_KEY_PCSRK_STR;
        break;
    case BTM_LE_KEY_LENC:
        name = BTC_BLE_STORAGE_LE_KEY_LENC_STR;
        break;
    case BTM_LE_KEY_LCSRK:
        name = BTC_BLE_STORAGE_LE_KEY_LCSRK_STR;
        break;
    case BTM_LE_KEY_LID:
        name =  BTC_BLE_STORAGE_LE_KEY_LID_STR;
    default:
        return BT_STATUS_FAIL;
    }
    size_t length = key_length;
    int ret = btc_config_get_bin(bdstr, name, (uint8_t *)key_value, &length);
    return ret ? BT_STATUS_SUCCESS : BT_STATUS_FAIL;

}

static int btc_storage_get_num_ble_bond_devices(void)
{
    int num_dev = 0;
    uint32_t device_type = 0;

    btc_config_lock();
    for (const btc_config_section_iter_t *iter = btc_config_section_begin(); iter != btc_config_section_end();
            iter = btc_config_section_next(iter)) {
        const char *name = btc_config_section_name(iter);
        if (!string_is_bdaddr(name) ||
                !btc_config_get_int(name, BTC_BLE_STORAGE_DEV_TYPE_STR, (int *)&device_type) ||
                !(device_type & BT_DEVICE_TYPE_BLE)) {
            continue;
        }

        num_dev++;
    }
    btc_config_unlock();

    return num_dev;
}

static int btc_storage_get_bonded_ble_devices_list(esp_ble_bond_dev_t *bond_dev, int dev_num)
{
    bt_bdaddr_t bd_addr;
    char buffer[sizeof(tBTM_LE_KEY_VALUE)] = {0};

    btc_config_lock();
    for (const btc_config_section_iter_t *iter = btc_config_section_begin(); iter != btc_config_section_end();
            iter = btc_config_section_next(iter)) {

        if (dev_num-- <= 0) {
            break;
        }
        uint32_t device_type = 0;
        const char *name = btc_config_section_name(iter);

        if (!string_is_bdaddr(name) ||
                !btc_config_get_int(name, BTC_BLE_STORAGE_DEV_TYPE_STR, (int *)&device_type) ||
                !(device_type & BT_DEVICE_TYPE_BLE)) {
            dev_num ++;
            continue;
        }

        //      #define BTM_LE_KEY_PENC      SMP_SEC_KEY_TYPE_ENC        /* encryption information of peer device */
        //      #define BTM_LE_KEY_PID       SMP_SEC_KEY_TYPE_ID         /* identity key of the peer device */
        //      #define BTM_LE_KEY_PCSRK     SMP_SEC_KEY_TYPE_CSRK      /* peer SRK */
        //#define BTM_LE_KEY_PLK       SMP_SEC_KEY_TYPE_LK
        //#define BTM_LE_KEY_LLK       (SMP_SEC_KEY_TYPE_LK << 4)
        //      #define BTM_LE_KEY_LENC      (SMP_SEC_KEY_TYPE_ENC << 4)  /* master role security information:div */
        //#define BTM_LE_KEY_LID       (SMP_SEC_KEY_TYPE_ID << 4)   /* master device ID key */
        //      #define BTM_LE_KEY_LCSRK     (SMP_SEC_KEY_TYPE_CSRK << 4) /* local CSRK has been deliver to peer */

        string_to_bdaddr(name, &bd_addr);
        memcpy(bond_dev->bd_addr, bd_addr.address, sizeof(bt_bdaddr_t));
        //resolve the peer device long term key
        if (_btc_storage_get_ble_bonding_key(&bd_addr, BTM_LE_KEY_PENC, buffer, sizeof(tBTM_LE_PENC_KEYS)) == BT_STATUS_SUCCESS) {
            bond_dev->bond_key.key_mask |= ESP_BLE_ENC_KEY_MASK;
            memcpy(&bond_dev->bond_key.penc_key, buffer, sizeof(tBTM_LE_PENC_KEYS));
        }
        //resolve the peer device csrk
        if (_btc_storage_get_ble_bonding_key(&bd_addr, BTM_LE_KEY_PCSRK, buffer, sizeof(tBTM_LE_PCSRK_KEYS)) == BT_STATUS_SUCCESS) {
            bond_dev->bond_key.key_mask |= ESP_BLE_CSR_KEY_MASK;
            memcpy(&bond_dev->bond_key.pcsrk_key, buffer, sizeof(tBTM_LE_PCSRK_KEYS));
        }
        //resolve the peer device irk
        if (_btc_storage_get_ble_bonding_key(&bd_addr, BTM_LE_KEY_PID, buffer, sizeof(tBTM_LE_PID_KEYS)) == BT_STATUS_SUCCESS) {
            bond_dev->bond_key.key_mask |= ESP_BLE_ID_KEY_MASK;
            tBTM_LE_PID_KEYS *pid_key = (tBTM_LE_PID_KEYS *) buffer;
            //Note: The memory size of the pid_key.addr_type in bond_key is different from that of (tBTM_LE_PID_KEYS) *pid_key.
            memcpy(&bond_dev->bond_key.pid_key.irk, pid_key->irk, ESP_BT_OCTET16_LEN);
            bond_dev->bond_key.pid_key.addr_type = pid_key->addr_type;
            memcpy(&bond_dev->bond_key.pid_key.static_addr, pid_key->static_addr, ESP_BD_ADDR_LEN);
        }

        //resolve the local device long term key
        if (_btc_storage_get_ble_bonding_key(&bd_addr, BTM_LE_KEY_LENC, buffer, sizeof(tBTM_LE_LENC_KEYS)) == BT_STATUS_SUCCESS) {
            //bond_dev->bond_key.key_mask |= ESP_BLE_ENC_KEY_MASK;
            memcpy(&bond_dev->bond_key.lenck_key, buffer, sizeof(tBTM_LE_LENC_KEYS));
        }
        //resolve the local device csrk
        if (_btc_storage_get_ble_bonding_key(&bd_addr, BTM_LE_KEY_LCSRK, buffer, sizeof(tBTM_LE_LCSRK_KEYS)) == BT_STATUS_SUCCESS) {
            //bond_dev->bond_key.key_mask |= ESP_BLE_CSR_KEY_MASK;
            memcpy(&bond_dev->bond_key.lcsrk_key, buffer, sizeof(tBTM_LE_LCSRK_KEYS));
        }
        if (_btc_storage_get_ble_bonding_key(&bd_addr, BTM_LE_KEY_LID, buffer, sizeof(tBTM_LE_PID_KEYS)) == BT_STATUS_SUCCESS) {
            bond_dev->bond_key.key_mask |= ESP_BLE_ID_KEY_MASK;
            tBTM_LE_PID_KEYS *lid_key = (tBTM_LE_PID_KEYS *) buffer;
            //Note: The memory size of the pid_key.addr_type in bond_key is different from that of (tBTM_LE_PID_KEYS) *pid_key.
            memcpy(&bond_dev->bond_key.lid_key.irk, lid_key->irk, ESP_BT_OCTET16_LEN);
            bond_dev->bond_key.lid_key.addr_type = lid_key->addr_type;
            memcpy(&bond_dev->bond_key.lid_key.static_addr, lid_key->static_addr, ESP_BD_ADDR_LEN);
        }

        //search for the next bond device
        bond_dev++;
    }
    btc_config_unlock();

    return 0;
}

#define CONFIG_FILE_MAX_SIZE             (1536)//1.5k
#define CONFIG_FILE_DEFAULE_LENGTH       (2048)
#define CONFIG_KEY                       "bt_cfg_key"
#define CONFIG_DEFAULT_SECTION "Global"

static int get_config_size_from_flash(nvs_handle_t fp)
{
    assert(fp != 0);

    esp_err_t err;
    const size_t keyname_bufsz = sizeof(CONFIG_KEY) + 5 + 1; // including log10(sizeof(i))
    char *keyname = osi_calloc(keyname_bufsz);
    if (!keyname){
        ESP_LOGE("OSI", "%s, malloc error\n", __func__);
        return 0;
    }
    size_t length = CONFIG_FILE_DEFAULE_LENGTH;
    size_t total_length = 0;
    uint16_t i = 0;
    snprintf(keyname, keyname_bufsz, "%s%d", CONFIG_KEY, 0);
    err = nvs_get_blob(fp, keyname, NULL, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        osi_free(keyname);
        return 0;
    }
    if (err != ESP_OK) {
        ESP_LOGE("OSI", "%s, error %d\n", __func__, err);
        osi_free(keyname);
        return 0;
    }
    total_length += length;
    while (length == CONFIG_FILE_MAX_SIZE) {
        length = CONFIG_FILE_DEFAULE_LENGTH;
        snprintf(keyname, keyname_bufsz, "%s%d", CONFIG_KEY, ++i);
        err = nvs_get_blob(fp, keyname, NULL, &length);

        if (err == ESP_ERR_NVS_NOT_FOUND) {
            break;
        }
        if (err != ESP_OK) {
            ESP_LOGE("OSI", "%s, error %d\n", __func__, err);
            osi_free(keyname);
            return 0;
        }
        total_length += length;
    }
    osi_free(keyname);
    return total_length;
}

static void config_parse(nvs_handle_t fp, config_t *config)
{
    assert(fp != 0);
    assert(config != NULL);

    esp_err_t err;
    int line_num = 0;
    int err_code = 0;
    uint16_t i = 0;
    size_t length = CONFIG_FILE_DEFAULE_LENGTH;
    size_t total_length = 0;
    char *line = osi_calloc(1024);
    char *section = osi_calloc(1024);
    const size_t keyname_bufsz = sizeof(CONFIG_KEY) + 5 + 1; // including log10(sizeof(i))
    char *keyname = osi_calloc(keyname_bufsz);
    int buf_size = get_config_size_from_flash(fp);
    char *buf = osi_calloc(buf_size);
    if(buf_size == 0) { //First use nvs
        goto error;
    }
    if (!line || !section || !buf || !keyname) {
        err_code |= 0x01;
        goto error;
    }
    snprintf(keyname, keyname_bufsz, "%s%d", CONFIG_KEY, 0);
    err = nvs_get_blob(fp, keyname, buf, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        goto error;
    }
    if (err != ESP_OK) {
        err_code |= 0x02;
        goto error;
    }
    total_length += length;
    while (length == CONFIG_FILE_MAX_SIZE) {
        length = CONFIG_FILE_DEFAULE_LENGTH;
        snprintf(keyname, keyname_bufsz, "%s%d", CONFIG_KEY, ++i);
        err = nvs_get_blob(fp, keyname, buf + CONFIG_FILE_MAX_SIZE * i, &length);

        if (err == ESP_ERR_NVS_NOT_FOUND) {
            break;
        }
        if (err != ESP_OK) {
            err_code |= 0x02;
            goto error;
        }
        total_length += length;
    }
    char *p_line_end;
    char *p_line_bgn = buf;
    strcpy(section, CONFIG_DEFAULT_SECTION);

    while ( (p_line_bgn < buf + total_length - 1) && (p_line_end = strchr(p_line_bgn, '\n'))) {

        // get one line
        int line_len = p_line_end - p_line_bgn;
        if (line_len > 1023) {
            ESP_LOGD("OSI", "%s exceed max line length on line %d.\n", __func__, line_num);
            break;
        }
        memcpy(line, p_line_bgn, line_len);
        line[line_len] = '\0';
        p_line_bgn = p_line_end + 1;
        char *line_ptr = trim(line);
        ++line_num;

        // Skip blank and comment lines.
        if (*line_ptr == '\0' || *line_ptr == '#') {
            continue;
        }

        if (*line_ptr == '[') {
            size_t len = strlen(line_ptr);
            if (line_ptr[len - 1] != ']') {
                ESP_LOGD("OSI", "%s unterminated section name on line %d.\n", __func__, line_num);
                continue;
            }
            strncpy(section, line_ptr + 1, len - 2);
            section[len - 2] = '\0';
        } else {
            char *split = strchr(line_ptr, '=');
            if (!split) {
                ESP_LOGD("OSI", "%s no key/value separator found on line %d.\n", __func__, line_num);
                continue;
            }
            *split = '\0';
            config_set_string(config, section, trim(line_ptr), trim(split + 1), true);
        }
    }

error:
    if (buf) {
        free(buf);
    }
    if (line) {
        free(line);
    }
    if (section) {
        free(section);
    }
    if (keyname) {
        free(keyname);
    }
    if (err_code) {
        ESP_LOGE("OSI", "%s returned with err code: %d\n", __func__, err_code);
    }
}

config_t *config_new(const char *filename)
{
    assert(filename != NULL);

    config_t *config = config_new_empty();
    if (!config) {
        return NULL;
    }

    esp_err_t err;
    nvs_handle_t fp;
    err = nvs_open(filename, NVS_READWRITE, &fp);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
            ESP_LOGE("OSI", "%s: NVS not initialized. "
                      "Call nvs_flash_init before initializing bluetooth.", __func__);
        } else {
            ESP_LOGE("OSI", "%s unable to open NVS namespace '%s'\n", __func__, filename);
        }
        config_free(config);
        return NULL;
    }

    config_parse(fp, config);
    nvs_close(fp);
    return config;
}

static int get_config_size(const config_t *config)
{
    assert(config != NULL);

    int w_len = 0, total_size = 0;

    for (const list_node_t *node = list_begin(config->sections); node != list_end(config->sections); node = list_next(node)) {
        const section_t *section = (const section_t *)list_node(node);
        w_len = strlen(section->name) + strlen("[]\n");// format "[section->name]\n"
        total_size += w_len;

        for (const list_node_t *enode = list_begin(section->entries); enode != list_end(section->entries); enode = list_next(enode)) {
            const entry_t *entry = (const entry_t *)list_node(enode);
            w_len = strlen(entry->key) + strlen(entry->value) + strlen(" = \n");// format "entry->key = entry->value\n"
            total_size += w_len;
        }

        // Only add a separating newline if there are more sections.
        if (list_next(node) != list_end(config->sections)) {
                total_size ++;  //'\n'
        } else {
            break;
        }
    }
    total_size ++; //'\0'
    return total_size;
}

bool config_save(const config_t *config, const char *filename)
{
    assert(config != NULL);
    assert(filename != NULL);
    assert(*filename != '\0');

    esp_err_t err;
    int err_code = 0;
    nvs_handle_t fp;
    char *line = osi_calloc(1024);
    const size_t keyname_bufsz = sizeof(CONFIG_KEY) + 5 + 1; // including log10(sizeof(i))
    char *keyname = osi_calloc(keyname_bufsz);
    int config_size = get_config_size(config);
    char *buf = osi_calloc(config_size);
    if (!line || !buf || !keyname) {
        err_code |= 0x01;
        goto error;
    }

    err = nvs_open(filename, NVS_READWRITE, &fp);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
            ESP_LOGE("OSI", "%s: NVS not initialized. "
                      "Call nvs_flash_init before initializing bluetooth.", __func__);
        }
        err_code |= 0x02;
        goto error;
    }

    int w_cnt, w_cnt_total = 0;
    for (const list_node_t *node = list_begin(config->sections); node != list_end(config->sections); node = list_next(node)) {
        const section_t *section = (const section_t *)list_node(node);
        w_cnt = snprintf(line, 1024, "[%s]\n", section->name);
        if(w_cnt < 0) {
            ESP_LOGE("OSI", "snprintf error w_cnt %d.",w_cnt);
            err_code |= 0x10;
            goto error;
        }
        if(w_cnt_total + w_cnt > config_size) {
            ESP_LOGE("OSI", "%s, memcpy size (w_cnt + w_cnt_total = %d) is larger than buffer size (config_size = %d).", __func__, (w_cnt + w_cnt_total), config_size);
            err_code |= 0x20;
            goto error;
        }
        ESP_LOGD("OSI", "section name: %s, w_cnt + w_cnt_total = %d\n", section->name, w_cnt + w_cnt_total);
        memcpy(buf + w_cnt_total, line, w_cnt);
        w_cnt_total += w_cnt;

        for (const list_node_t *enode = list_begin(section->entries); enode != list_end(section->entries); enode = list_next(enode)) {
            const entry_t *entry = (const entry_t *)list_node(enode);
            ESP_LOGD("OSI", "(key, val): (%s, %s)\n", entry->key, entry->value);
            w_cnt = snprintf(line, 1024, "%s = %s\n", entry->key, entry->value);
            if(w_cnt < 0) {
                ESP_LOGE("OSI", "snprintf error w_cnt %d.",w_cnt);
                err_code |= 0x10;
                goto error;
            }
            if(w_cnt_total + w_cnt > config_size) {
                ESP_LOGE("OSI", "%s, memcpy size (w_cnt + w_cnt_total = %d) is larger than buffer size.(config_size = %d)", __func__, (w_cnt + w_cnt_total), config_size);
                err_code |= 0x20;
                goto error;
            }
            ESP_LOGD("OSI", "%s, w_cnt + w_cnt_total = %d", __func__, w_cnt + w_cnt_total);
            memcpy(buf + w_cnt_total, line, w_cnt);
            w_cnt_total += w_cnt;
        }

        // Only add a separating newline if there are more sections.
        if (list_next(node) != list_end(config->sections)) {
            buf[w_cnt_total] = '\n';
            w_cnt_total += 1;
        } else {
            break;
        }
    }
    buf[w_cnt_total] = '\0';
    if (w_cnt_total < CONFIG_FILE_MAX_SIZE) {
        snprintf(keyname, keyname_bufsz, "%s%d", CONFIG_KEY, 0);
        err = nvs_set_blob(fp, keyname, buf, w_cnt_total);
        if (err != ESP_OK) {
            nvs_close(fp);
            err_code |= 0x04;
            goto error;
        }
    }else {
        int count = (w_cnt_total / CONFIG_FILE_MAX_SIZE);
        assert(count <= 0xFF);
        for (uint8_t i = 0; i <= count; i++)
        {
            snprintf(keyname, keyname_bufsz, "%s%d", CONFIG_KEY, i);
            if (i == count) {
                err = nvs_set_blob(fp, keyname, buf + i*CONFIG_FILE_MAX_SIZE, w_cnt_total - i*CONFIG_FILE_MAX_SIZE);
                ESP_LOGD("OSI", "save keyname = %s, i = %d, %d\n", keyname, i, w_cnt_total - i*CONFIG_FILE_MAX_SIZE);
            }else {
                err = nvs_set_blob(fp, keyname, buf + i*CONFIG_FILE_MAX_SIZE, CONFIG_FILE_MAX_SIZE);
                ESP_LOGD("OSI", "save keyname = %s, i = %d, %d\n", keyname, i, CONFIG_FILE_MAX_SIZE);
            }
            if (err != ESP_OK) {
                nvs_close(fp);
                err_code |= 0x04;
                goto error;
            }
        }
    }

    err = nvs_commit(fp);
    if (err != ESP_OK) {
        nvs_close(fp);
        err_code |= 0x08;
        goto error;
    }

    nvs_close(fp);
    osi_free(line);
    osi_free(buf);
    osi_free(keyname);
    return true;

error:
    if (buf) {
        osi_free(buf);
    }
    if (line) {
        osi_free(line);
    }
    if (keyname) {
        osi_free(keyname);
    }
    if (err_code) {
        ESP_LOGE("OSI", "%s, err_code: 0x%x\n", __func__, err_code);
    }
    return false;
}

static void memcpy_reversed(void* dest, const void* src, size_t size)
{
    uint8_t* d = (uint8_t*)dest;
    uint8_t* s = (uint8_t*)src;

    for (int i = 0; i < size; i++)
    {
        d[(size - 1) - i] = s[i];
    }
}

static bool check_is_present(const void* src, size_t size)
{
    uint8_t* s = (uint8_t*)src;

    for (int i = 0; i < size; i++)
    {
        if (s[i] != 0)
        {
            return true;
        }
    }

    return false;
}

static void bluedroid_nvs_to_nimble_nvs(void)
{
    int peer_count = 0;
    int our_count = 0;
    int cccd_count = 0;
    int dev_count = 0;
    ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &peer_count);
    ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &our_count);
    ble_store_util_count(BLE_STORE_OBJ_TYPE_CCCD, &cccd_count);
    ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, &dev_count);

    if (peer_count > 0 || our_count > 0)
    {
        ESP_LOGI(TAG, "Found data in Nimble NVS: %d peer sec, %d our sec, %d ccid, %d dev rec", peer_count, our_count, cccd_count, dev_count);
        return;
    }

    static const char* FILE_NAME = "bt_config.conf";
    config = config_new(FILE_NAME);
    if (!config)
    {
        ESP_LOGI(TAG, "Did not find any Bluedroid NVS data.");
        config_free(config);
        return;
    }

    static esp_ble_bond_dev_t devices[10];
    int device_count = btc_storage_get_num_ble_bond_devices();
    btc_storage_get_bonded_ble_devices_list(devices, device_count);

    ESP_LOGI(TAG, "Found %u devices in Bluedroid NVS.", (unsigned int)device_count);

    for (int i = 0; i < device_count; i++)
    {
        ESP_LOGI(TAG, "Converting bond for %02X:%02X:%02X:%02X:%02X:%02X",
            devices[i].bd_addr[0],
            devices[i].bd_addr[1],
            devices[i].bd_addr[2],
            devices[i].bd_addr[3],
            devices[i].bd_addr[4],
            devices[i].bd_addr[5]);

        static struct ble_store_value_sec sec;

        // Default everything to zero.
        memset(&sec, 0, sizeof(sec));

        // Copy over the address.
        sec.peer_addr.type = BLE_ADDR_TYPE_RANDOM;
        memcpy_reversed(sec.peer_addr.val, devices[i].bond_key.pid_key.static_addr, sizeof(sec.peer_addr.val));

        // Copy over the CSRK, if present.
        if (check_is_present(devices[i].bond_key.pcsrk_key.csrk, sizeof(sec.csrk)))
        {
            sec.csrk_present = 1;
            memcpy(sec.csrk, devices[i].bond_key.pcsrk_key.csrk, sizeof(sec.csrk));
        }

        // Copy over the LTK, if present--it should be!
        if (check_is_present(devices[i].bond_key.penc_key.ltk, sizeof(sec.ltk)))
        {
            sec.ltk_present = 1;
            sec.key_size = devices[i].bond_key.penc_key.key_size;

            memcpy(&sec.ediv, &devices[i].bond_key.penc_key.ediv, sizeof(sec.ediv));
            memcpy(&sec.rand_num, devices[i].bond_key.penc_key.rand, sizeof(sec.rand_num));
            memcpy(sec.ltk, devices[i].bond_key.penc_key.ltk, sizeof(sec.ltk));
        }

        // Copy over the IRK, which seems to exist even when we don't use authentication.
        if (check_is_present(devices[i].bond_key.pid_key.irk, sizeof(sec.irk)))
        {
            sec.irk_present = 1;
            memcpy(sec.irk, devices[i].bond_key.pid_key.irk, sizeof(sec.irk));
        }

        sec.sc = 1;
        sec.authenticated = 0;

        // Write this directly to the NV store, to be processed later.
        int rc = ble_store_write(BLE_STORE_OBJ_TYPE_PEER_SEC, (union ble_store_value*) &sec);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "ble_store_write() failed for peer data with error code %d\n", rc);
        }

        // Default everything to zero.
        memset(&sec, 0, sizeof(sec));

        // Copy over the address.
        sec.peer_addr.type = BLE_ADDR_TYPE_RANDOM;
        memcpy_reversed(sec.peer_addr.val, devices[i].bond_key.pid_key.static_addr, sizeof(sec.peer_addr.val));

        // I don't think CSRK is normally present, but copy if it is.
        if (check_is_present(devices[i].bond_key.lcsrk_key.csrk, sizeof(sec.csrk)))
        {
            sec.csrk_present = 1;
            memcpy(sec.csrk, devices[i].bond_key.lcsrk_key.csrk, sizeof(sec.csrk));
            // Not sure what to do with div and counter.
        }

        // Copy the LTK.  Not sure if this one or the peer LTK matters.
        if (check_is_present(devices[i].bond_key.lenck_key.ltk, sizeof(sec.ltk)))
        {
            sec.ltk_present = 1;

            sec.key_size = devices[i].bond_key.lenck_key.key_size;
            sec.ediv = devices[i].bond_key.lenck_key.div;
            memcpy(sec.ltk, devices[i].bond_key.lenck_key.ltk, sizeof(sec.ltk));
            // Not sure what to do with random number.
        }

        // I don't think the IRK is normally present, but copy if it is.
        if (check_is_present(devices[i].bond_key.lid_key.irk, sizeof(sec.irk)))
        {
            sec.irk_present = 1;
            memcpy(sec.irk, devices[i].bond_key.lid_key.irk, sizeof(sec.irk));
            // Not sure what to do with static address.

        }

        sec.sc = 1;
        sec.authenticated = 0;

        // Write this directly to the NV store, to be processed later.
        rc = ble_store_write(BLE_STORE_OBJ_TYPE_OUR_SEC, (union ble_store_value*) &sec);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "ble_store_write() failed for our data with error code %d\n", rc);
        }
    }

    // Erase all the old Bluedroid data so it doesn't get loaded again in the future.
    config_free(config);

    config = config_new_empty();
    if (!config) {
        return;
    }

    config_save(config, FILE_NAME);
    config_free(config);

    return;
}

/***************************************************************************************/
#endif /* MYNEWT_VAL(BLE_STORE_CONFIG_PERSIST) */
