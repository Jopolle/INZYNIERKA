#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>

#include <lvgl.h>

#if defined(CONFIG_FAT_FILESYSTEM_ELM)
#include <ff.h>
#endif

#include <stdio.h>

LOG_MODULE_REGISTER(main);

#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT   "/" DISK_DRIVE_NAME ":"

#define TEST_FILE_NAME  "test.txt"
#define READ_BUF_SIZE   128

static const char *disk_mount_pt = DISK_MOUNT_PT;

#if defined(CONFIG_FAT_FILESYSTEM_ELM)
static FATFS fat_fs;
#endif

static struct fs_mount_t fs_mnt = {
#if defined(CONFIG_FAT_FILESYSTEM_ELM)
    .type = FS_FATFS,
    .fs_data = &fat_fs,
#else
#   error "This sample expects CONFIG_FAT_FILESYSTEM_ELM=y"
#endif
    .mnt_point   = DISK_MOUNT_PT,
    .storage_dev = (void *)DISK_DRIVE_NAME,
};

static const struct device *display_dev;
static lv_obj_t *status_label;
static bool fs_ready = false;

static void set_status(const char *txt)
{
    if (status_label) {
        lv_label_set_text(status_label, txt);
    }
    LOG_INF("%s", txt);
}

static int mount_sd(void)
{
    int res;

    if (fs_ready) {
        return 0;
    }

    LOG_INF("Mounting SD card on %s (disk \"%s\")", disk_mount_pt, DISK_DRIVE_NAME);

    uint32_t block_count;
    uint32_t block_size;

    if (disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_INIT, NULL) != 0) {
        LOG_ERR("disk init failed");
        set_status("Błąd inicjalizacji SD");
        return -EIO;
    }

    if (disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_GET_SECTOR_COUNT, &block_count) == 0 &&
        disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_GET_SECTOR_SIZE, &block_size) == 0) {
        uint64_t size_mb = ((uint64_t)block_count * block_size) >> 20;
        LOG_INF("SD size: %u MB", (uint32_t)size_mb);
    }

    res = fs_mount(&fs_mnt);
    if (res != 0) {
        LOG_ERR("fs_mount failed: %d", res);
        set_status("Nie mogę zamontować SD");
        return res;
    }

    fs_ready = true;
    set_status("SD zamontowana");
    return 0;
}

static void read_test_file(void)
{
    int ret;
    struct fs_file_t file;
    char path[64];
    char buf[READ_BUF_SIZE];
    int read_bytes;

    if (!fs_ready) {
        ret = mount_sd();
        if (ret != 0) {
            return;
        }
    }

    fs_file_t_init(&file);

    snprintf(path, sizeof(path), "%s/%s", disk_mount_pt, TEST_FILE_NAME);

    LOG_INF("Opening file: %s", path);

    ret = fs_open(&file, path, FS_O_READ);
    if (ret < 0) {
        LOG_ERR("fs_open(%s) failed: %d", path, ret);
        set_status("Nie mogę otworzyć test.txt");
        return;
    }

    read_bytes = fs_read(&file, buf, sizeof(buf) - 1);
    if (read_bytes < 0) {
        LOG_ERR("fs_read failed: %d", read_bytes);
        set_status("Błąd odczytu test.txt");
        fs_close(&file);
        return;
    }

    buf[read_bytes] = '\0';

    //LOG_INF("Read %d bytes: %s", read_bytes, log_strdup(buf));

    lv_label_set_text(status_label, buf);

    fs_close(&file);
}

static void btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        read_test_file();
    }
}

static void create_gui(void)
{

    lv_obj_t *scr = lv_scr_act();


    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, "Czekam na kliknięcie...");
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 10);


    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 200, 60);
    lv_obj_center(btn);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, "Otwórz test.txt");
    lv_obj_center(label);
}

int main(void)
{
    LOG_INF("LVGL + SD test.txt sample start");

    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        return 0;
    }

    display_blanking_off(display_dev);

    create_gui();

    while (1) {
        lv_timer_handler();
        k_sleep(K_MSEC(20));
    }

    return 0;
}

