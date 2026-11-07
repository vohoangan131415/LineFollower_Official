#ifndef INC_EDGE_FOLLOW_H_
#define INC_EDGE_FOLLOW_H_

#include <stdint.h>

/**
 * THU VIEN DO LINE BAM MEP (5 mat analog)
 *
 * Y tuong: thay vi giu TAM vach o giua xe, ta giu MOT MEP cua vach o dung vi tri
 * mong muon. Mep = cho chuyen tu trang sang den DAU TIEN khi quet tu phia NGOAI vao,
 * nen khi vach tach doi (nga ba chu Y) xe van bam dung nhanh da chon.
 *
 * Quy uoc (khop rpr220.c):
 *   - Mang 5 mat da chuan hoa 0..1000 (0 = trang, 1000 = den).
 *   - Mat 0 o ben PHAI (+2000), mat 4 o ben TRAI (-2000). Don vi: 1000 = 1 pitch.
 *   - Sai so DUONG = vach lech sang PHAI (xe can lai phai), cung dau RPR220_Get_Error.
 */

#define EDGE_NUM_SENSORS          5
#define EDGE_THRESH               500     // Nguong 0..1000 coi mot mat la "den"
#define EDGE_OUTSIDE_OFFSET       500     // Mep nam ngoai mang: cong them ~0.5 pitch ra ngoai

/* Diem dat mac dinh: |vi tri mep| khi xe nam chinh giua vach.
 * Ly thuyet = 13 mm / P(mm) * 1000. NEN DO THUC TE (xem huong dan) */
#define EDGE_SP_LEFT_DEFAULT      1300
#define EDGE_SP_RIGHT_DEFAULT     1300

typedef enum {
    EDGE_SIDE_LEFT  = -1,   // Bam mep TRAI  (vi du: nhanh tren, 1 kg)
    EDGE_SIDE_RIGHT = +1    // Bam mep PHAI  (vi du: nhanh duoi, 2 kg)
} EdgeSide_t;

/**
 * @brief  Khoi tao thu vien va dat diem dat cho 2 mep
 * @param  sp_left   Diem dat mep trai  (so duong, vi du 1300)
 * @param  sp_right  Diem dat mep phai  (so duong, vi du 1300)
 */
void EdgeFollow_Init(int16_t sp_left, int16_t sp_right);

/**
 * @brief  Dat lai sai so luu giu (dung khi chua bam mep, de dong bo luc chuyen che do)
 * @param  seed_error  Sai so hien tai (vi du line_pos_error tu RPR220_Get_Error)
 */
void EdgeFollow_Reset(int16_t seed_error);

/**
 * @brief  Tim vi tri mep bang cach quet tu ngoai vao va noi suy giua 2 mat
 * @param  norm   Mang 5 mat da chuan hoa 0..1000
 * @param  side   EDGE_SIDE_LEFT hoac EDGE_SIDE_RIGHT
 * @param  found  Ra: 1 neu thay mep, 0 neu khong mat nao den (mat line)
 * @retval Vi tri mep (1000 = 1 pitch, am = ben trai)
 */
int32_t EdgeFollow_GetPosition(const volatile uint16_t *norm, EdgeSide_t side, uint8_t *found);

/**
 * @brief  Tinh sai so bam mep dua vao PD. Mat line thi giu sai so cu.
 * @retval Sai so (duong = lai phai)
 */
int16_t EdgeFollow_GetError(const volatile uint16_t *norm, EdgeSide_t side);

/**
 * @brief  1 neu lan goi EdgeFollow_GetError gan nhat khong thay mep
 */
uint8_t EdgeFollow_IsLost(void);

#endif /* INC_EDGE_FOLLOW_H_ */